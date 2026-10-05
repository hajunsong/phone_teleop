"""Network helpers shared by the simulation: adb reverse for USB, UDP beacon for Wi-Fi.

USB  : the phone reaches the PC's MQTT broker at 127.0.0.1:PORT on the phone,
       through  adb reverse tcp:PORT tcp:PORT  (UsbReverser keeps it installed on
       every authorised phone, hot-plug included).
Wi-Fi: the phone needs the PC's address; Beacon broadcasts
       "HUMANOID_TELEOP <port> <host>" on UDP 5000 and the app's 'PC 찾기' listens.
"""

from __future__ import annotations

import socket
import threading
import time

import adb_tools


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
    """UDP broadcast  'HUMANOID_TELEOP <port> <host>'  so the app can find the broker."""

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
