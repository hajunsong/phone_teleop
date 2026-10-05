"""Minimal MQTT 3.1.1 broker and client (QoS 0), no dependencies.

Enough for the simulation loop  phone -> broker -> MuJoCo  on one PC without
installing mosquitto.  The real robot uses its own broker (192.168.0.142:1883);
the phone app and the simulation can point at that one instead.

    Broker(("0.0.0.0", 1883)).start()
    c = Client("sim"); c.connect("127.0.0.1", 1883); c.subscribe("/a/#", cb); c.publish("/a/b", b"x")

Broker: CONNECT/CONNACK, SUBSCRIBE/SUBACK (+ and # wildcards), PUBLISH fan-out
(QoS 1/2 publishes are acknowledged at the first step and delivered at QoS 0),
retained messages, PINGREQ, DISCONNECT.  No auth, no persistence, no wills.
"""

from __future__ import annotations

import socket
import struct
import threading
import time


# ---------------------------------------------------------------- framing

def _enc_len(n: int) -> bytes:
    out = bytearray()
    while True:
        d = n % 128
        n //= 128
        out.append(d | (0x80 if n else 0))
        if not n:
            return bytes(out)


def _enc_str(s: str) -> bytes:
    b = s.encode("utf-8")
    return struct.pack(">H", len(b)) + b


def _packet(header: int, body: bytes) -> bytes:
    return bytes([header]) + _enc_len(len(body)) + body


def _recv_exact(s: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = s.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("closed")
        buf += chunk
    return bytes(buf)


def _read_packet(s: socket.socket) -> tuple[int, bytes]:
    h = _recv_exact(s, 1)[0]
    mult, n = 1, 0
    while True:
        d = _recv_exact(s, 1)[0]
        n += (d & 0x7F) * mult
        if not d & 0x80:
            break
        mult *= 128
    return h, _recv_exact(s, n) if n else b""


def topic_matches(flt: str, topic: str) -> bool:
    f, t = flt.split("/"), topic.split("/")
    for i, part in enumerate(f):
        if part == "#":
            return True
        if i >= len(t) or (part != "+" and part != t[i]):
            return False
    return len(f) == len(t)


def _publish_packet(topic: str, payload: bytes, retain: bool = False) -> bytes:
    return _packet(0x30 | (1 if retain else 0), _enc_str(topic) + payload)


# ---------------------------------------------------------------- broker

class Broker:
    def __init__(self, addr=("0.0.0.0", 1883), log=print):
        self.addr = addr
        self.log = log
        self._lock = threading.Lock()
        self._subs: dict[socket.socket, set[str]] = {}
        self._wlock: dict[socket.socket, threading.Lock] = {}
        self._names: dict[socket.socket, str] = {}
        self._retained: dict[str, bytes] = {}
        self._srv: socket.socket | None = None
        self._running = False

    def start(self):
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 0)
        self._srv.bind(self.addr)                     # raises if the port is taken
        self._srv.listen(16)
        self._running = True
        threading.Thread(target=self._accept, daemon=True, name="mqtt-broker").start()

    def stop(self):
        self._running = False
        try:
            self._srv.close()
        except OSError:
            pass
        with self._lock:
            for s in list(self._subs):
                try:
                    s.close()
                except OSError:
                    pass

    def _accept(self):
        while self._running:
            try:
                c, a = self._srv.accept()
            except OSError:
                return
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            threading.Thread(target=self._client, args=(c, a), daemon=True).start()

    def _send(self, c: socket.socket, data: bytes):
        lk = self._wlock.get(c)
        if lk is None:
            return
        try:
            with lk:
                c.sendall(data)
        except OSError:
            pass

    def _client(self, c: socket.socket, addr):
        name = f"{addr[0]}:{addr[1]}"
        try:
            h, body = _read_packet(c)
            if h >> 4 != 1:
                return
            # CONNECT: skip protocol name / level / flags / keepalive, read client id
            pl = struct.unpack(">H", body[:2])[0]
            i = 2 + pl + 4
            cl = struct.unpack(">H", body[i:i + 2])[0]
            name = body[i + 2:i + 2 + cl].decode("utf-8", "replace") or name
            with self._lock:
                self._subs[c] = set()
                self._wlock[c] = threading.Lock()
                self._names[c] = name
            self._send(c, _packet(0x20, b"\x00\x00"))
            self.log(f"[mqtt] client '{name}' connected from {addr[0]}"
                     f"{'  (USB / adb reverse or local)' if addr[0].startswith('127.') else '  (network)'}")
            while self._running:
                h, body = _read_packet(c)
                t = h >> 4
                if t == 3:                                   # PUBLISH
                    qos = (h >> 1) & 3
                    tl = struct.unpack(">H", body[:2])[0]
                    topic = body[2:2 + tl].decode("utf-8", "replace")
                    off = 2 + tl
                    if qos:
                        pid = body[off:off + 2]
                        off += 2
                        self._send(c, _packet(0x40 if qos == 1 else 0x50, pid))
                    payload = body[off:]
                    if h & 1:
                        with self._lock:
                            if payload:
                                self._retained[topic] = payload
                            else:
                                self._retained.pop(topic, None)
                    pkt = _publish_packet(topic, payload)
                    with self._lock:
                        targets = [s for s, fl in self._subs.items() if any(topic_matches(f, topic) for f in fl)]
                    for s in targets:
                        self._send(s, pkt)
                elif t == 8:                                 # SUBSCRIBE
                    pid = body[:2]
                    i, filters, codes = 2, [], bytearray()
                    while i < len(body):
                        fl = struct.unpack(">H", body[i:i + 2])[0]
                        filters.append(body[i + 2:i + 2 + fl].decode("utf-8", "replace"))
                        i += 2 + fl + 1
                        codes.append(0)
                    with self._lock:
                        self._subs[c].update(filters)
                        ret = [(tp, pl_) for tp, pl_ in self._retained.items()
                               if any(topic_matches(f, tp) for f in filters)]
                    self._send(c, _packet(0x90, pid + bytes(codes)))
                    for tp, pl_ in ret:
                        self._send(c, _publish_packet(tp, pl_, retain=True))
                elif t == 10:                                # UNSUBSCRIBE
                    pid = body[:2]
                    i = 2
                    with self._lock:
                        while i < len(body):
                            fl = struct.unpack(">H", body[i:i + 2])[0]
                            self._subs[c].discard(body[i + 2:i + 2 + fl].decode("utf-8", "replace"))
                            i += 2 + fl
                    self._send(c, _packet(0xB0, pid))
                elif t == 12:                                # PINGREQ
                    self._send(c, _packet(0xD0, b""))
                elif t == 14:                                # DISCONNECT
                    break
        except (ConnectionError, OSError, struct.error, IndexError):
            pass
        finally:
            with self._lock:
                known = c in self._subs
                self._subs.pop(c, None)
                self._wlock.pop(c, None)
                self._names.pop(c, None)
            try:
                c.close()
            except OSError:
                pass
            if known:
                self.log(f"[mqtt] client '{name}' disconnected")


# ---------------------------------------------------------------- client

class Client:
    def __init__(self, client_id: str, keepalive: int = 30):
        self.client_id = client_id
        self.keepalive = keepalive
        self._s: socket.socket | None = None
        self._wlock = threading.Lock()
        self._cbs: list[tuple[str, callable]] = []
        self._pid = 0
        self.connected = False
        self._last_tx = 0.0

    def connect(self, host: str, port: int = 1883, timeout: float = 3.0):
        s = socket.create_connection((host, port), timeout=timeout)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        body = (_enc_str("MQTT") + bytes([4, 0x02]) + struct.pack(">H", self.keepalive)
                + _enc_str(self.client_id))
        s.sendall(_packet(0x10, body))
        h, ack = _read_packet(s)
        if h >> 4 != 2 or len(ack) < 2 or ack[1] != 0:
            s.close()
            raise ConnectionError(f"broker refused ({ack!r})")
        s.settimeout(None)
        self._s = s
        self.connected = True
        self._last_tx = time.monotonic()
        threading.Thread(target=self._reader, daemon=True, name="mqtt-client").start()
        threading.Thread(target=self._pinger, daemon=True, name="mqtt-ping").start()

    def _write(self, data: bytes):
        with self._wlock:
            self._s.sendall(data)
            self._last_tx = time.monotonic()

    def subscribe(self, flt: str, cb):
        self._cbs.append((flt, cb))
        self._pid = self._pid % 65535 + 1
        self._write(_packet(0x82, struct.pack(">H", self._pid) + _enc_str(flt) + b"\x00"))

    def publish(self, topic: str, payload: bytes, retain: bool = False):
        if self.connected:
            try:
                self._write(_publish_packet(topic, payload, retain))
            except OSError:
                self.connected = False

    def close(self):
        if self._s is None:
            return
        try:
            self._write(_packet(0xE0, b""))
        except OSError:
            pass
        self.connected = False
        try:
            self._s.close()
        except OSError:
            pass

    def _reader(self):
        try:
            while self.connected:
                h, body = _read_packet(self._s)
                if h >> 4 == 3:
                    tl = struct.unpack(">H", body[:2])[0]
                    topic = body[2:2 + tl].decode("utf-8", "replace")
                    off = 2 + tl + (2 if (h >> 1) & 3 else 0)
                    for flt, cb in list(self._cbs):
                        if topic_matches(flt, topic):
                            cb(topic, body[off:])
        except (ConnectionError, OSError):
            pass
        self.connected = False

    def _pinger(self):
        while self.connected:
            time.sleep(1.0)
            if time.monotonic() - self._last_tx > self.keepalive / 2:
                try:
                    self._write(_packet(0xC0, b""))
                except OSError:
                    self.connected = False
