"""Receive jog-button state from the phones over TCP (USB via adb reverse, or Wi-Fi).

One TCP server serves both transports:

    USB    phone -> 127.0.0.1:PORT (on the phone) --adb reverse--> PC:PORT
    Wi-Fi  phone -> PC_IP:PORT

so the only USB-specific part is keeping  adb reverse  installed on every
authorised phone (UsbReverser), and the only Wi-Fi-specific part is the UDP
beacon that lets the app find the PC's address (Beacon).

The phone uses no motion sensors.  It sends, at 50 Hz, which jog buttons are
held (-1/0/+1 per axis) for both arms in one line, a speed level and the jog
frame.  Line
protocol: see android/src/com/humanoid/teleop/MainActivity.java.
"""

from __future__ import annotations

import copy
import socket
import threading
import time
from dataclasses import dataclass, field

import numpy as np

import adb_tools


@dataclass
class PhoneTrack:
    peer: str
    model: str = "?"
    speed: int = 1               # 0 slow, 1 normal, 2 fast
    tool_frame: bool = False     # jog axes: False base frame, True tool frame
    # per arm: 6 axes (vx vy vz wx wy wz), each -1/0/+1
    jog: dict = field(default_factory=lambda: {"RA": np.zeros(6), "LA": np.zeros(6)})
    t_rx: float = 0.0            # host time of the last K line (watchdog; 0 = none yet)
    n_rx: int = 0
    events: list = field(default_factory=list)


class PhoneServer:
    """Threaded TCP server; `snapshot()` gives the main loop a consistent copy."""

    def __init__(self, port: int, bind: str = "0.0.0.0", log=print):
        self.port = port
        self.bind = bind
        self.log = log
        self._lock = threading.Lock()
        self._tracks: dict[int, PhoneTrack] = {}
        self._socks: dict[int, socket.socket] = {}
        self._next_id = 1
        self._srv: socket.socket | None = None
        self._running = False

    # ------------------------------------------------------------------ public
    def start(self):
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((self.bind, self.port))
        self._srv.listen(8)
        self._running = True
        threading.Thread(target=self._accept_loop, daemon=True, name="accept").start()

    def stop(self):
        self._running = False
        try:
            self._srv.close()
        except Exception:                               # noqa: BLE001
            pass
        with self._lock:
            for s in self._socks.values():
                try:
                    s.close()
                except Exception:                       # noqa: BLE001
                    pass

    def snapshot(self) -> dict[int, PhoneTrack]:
        """Copies of all live tracks; events are drained (delivered once)."""
        with self._lock:
            out = {}
            for k, t in self._tracks.items():
                out[k] = copy.deepcopy(t)
                t.events.clear()
            return out

    def send_status(self, cid: int, text: str):
        with self._lock:
            s = self._socks.get(cid)
        if s is None:
            return
        try:
            s.sendall(("ST " + text.replace("\n", " ") + "\n").encode("utf-8"))
        except OSError:
            pass

    # ------------------------------------------------------------------ internals
    def _accept_loop(self):
        while self._running:
            try:
                c, addr = self._srv.accept()
            except OSError:
                break
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self._lock:
                cid = self._next_id
                self._next_id += 1
                peer = f"{addr[0]}:{addr[1]}"
                self._tracks[cid] = PhoneTrack(peer=peer)
                self._socks[cid] = c
            self.log(f"[net] phone #{cid} connected from {peer}"
                     f"{'  (USB / adb reverse)' if addr[0].startswith('127.') else '  (Wi-Fi)'}")
            threading.Thread(target=self._client_loop, args=(cid, c), daemon=True,
                             name=f"phone{cid}").start()

    def _client_loop(self, cid: int, c: socket.socket):
        buf = b""
        try:
            while self._running:
                chunk = c.recv(65536)
                if not chunk:
                    break
                buf += chunk
                *lines, buf = buf.split(b"\n")
                for ln in lines:
                    self._handle(cid, ln.decode("utf-8", "replace").strip())
        except OSError:
            pass
        finally:
            with self._lock:
                t = self._tracks.pop(cid, None)
                self._socks.pop(cid, None)
            try:
                c.close()
            except OSError:
                pass
            self.log(f"[net] phone #{cid} disconnected ({t.peer if t else '?'})")

    def _handle(self, cid: int, line: str):
        if not line:
            return
        tok = line.split()
        with self._lock:
            t = self._tracks.get(cid)
            if t is None:
                return
            try:
                if tok[0] == "K" and len(tok) >= 16:
                    t.speed = int(tok[2])
                    t.tool_frame = tok[3] == "T"
                    ax = np.clip([float(v) for v in tok[4:16]], -1.0, 1.0)
                    t.jog = {"RA": ax[:6], "LA": ax[6:]}
                    t.t_rx = time.perf_counter()
                    t.n_rx += 1
                elif tok[0] == "H":
                    t.model = tok[2] if len(tok) > 2 else "?"
                    self.log(f"[net] phone #{cid}: {t.model}")
                    if tok[1] != "4":
                        self.log(f"[net] phone #{cid}: old app (protocol {tok[1]}) - reinstall the jog app")
                elif tok[0] == "E" and len(tok) >= 3:
                    t.events.append((tok[1], tok[2]))
            except (ValueError, IndexError) as exc:
                self.log(f"[net] phone #{cid}: bad line ({exc}): {line[:80]}")


class UsbReverser:
    """Keep  adb reverse tcp:PORT tcp:PORT  on every authorised phone (hot-plug)."""

    def __init__(self, port: int, period: float = 2.0, log=print):
        self.port, self.period, self.log = port, period, log
        self._done: set[str] = set()
        self._warned: set[str] = set()
        self._running = False

    def start(self):
        self._running = True
        threading.Thread(target=self._loop, daemon=True, name="adb-reverse").start()

    def stop(self):
        self._running = False
        for s in list(self._done):
            adb_tools.unreverse(s, self.port)

    def _loop(self):
        while self._running:
            try:
                devs = adb_tools.devices()
            except Exception:                           # noqa: BLE001
                devs = []
            live = {s for s, st in devs if st == "device"}
            for s, st in devs:
                if st == "unauthorized" and s not in self._warned:
                    self._warned.add(s)
                    self.log(f"[usb] {s}: unauthorized - allow USB debugging on the phone")
            for s in live - self._done:
                if adb_tools.reverse(s, self.port):
                    self._done.add(s)
                    self.log(f"[usb] {s}: adb reverse tcp:{self.port} ready -> app 'USB' mode")
            self._done &= live                          # unplugged -> redo on replug
            time.sleep(self.period)


def local_ipv4() -> list[str]:
    ips = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass
    try:                                                # the address of the default route
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return sorted(ip for ip in ips if not ip.startswith("127."))


class Beacon:
    """UDP broadcast  'HUMANOID_TELEOP <port> <host>'  so the app can find the PC."""

    def __init__(self, tcp_port: int, beacon_port: int = 5000, period: float = 1.0):
        self.tcp_port, self.beacon_port, self.period = tcp_port, beacon_port, period
        self._running = False

    def start(self):
        self._running = True
        threading.Thread(target=self._loop, daemon=True, name="beacon").start()

    def stop(self):
        self._running = False

    def _loop(self):
        msg = f"HUMANOID_TELEOP {self.tcp_port} {socket.gethostname()}".encode()
        while self._running:
            for ip in local_ipv4():
                bcast = ".".join(ip.split(".")[:3] + ["255"])  # assumes /24, the common case
                try:
                    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                        s.bind((ip, 0))                 # go out of that interface
                        s.sendto(msg, (bcast, self.beacon_port))
                except OSError:
                    pass
            time.sleep(self.period)
