"""Scripted stand-in for the jog app, for testing the PC side without a phone.

    python fake_phone.py                    RA: Z+ 2 s, Y- 1 s, Rz+ 2 s (normal speed)
    python fake_phone.py --arm LA --tool    same for LA, in the tool frame
    python fake_phone.py --arm both         both arms at once
    python fake_phone.py --drop             stop sending mid-move (watchdog test)
    python fake_phone.py --host 192.168.0.10     over Wi-Fi

Speaks exactly the app's line protocol (K lines at 50 Hz).
"""

from __future__ import annotations

import argparse
import socket
import threading
import time

SCRIPT = [            # (seconds, vx vy vz wx wy wz)
    (0.5, (0, 0, 0, 0, 0, 0)),
    (2.0, (0, 0, 1, 0, 0, 0)),
    (1.0, (0, -1, 0, 0, 0, 0)),
    (2.0, (0, 0, 0, 0, 0, 1)),
    (0.5, (0, 0, 0, 0, 0, 0)),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5001)
    ap.add_argument("--arm", default="RA")
    ap.add_argument("--speed", type=int, default=1)
    ap.add_argument("--tool", action="store_true")
    ap.add_argument("--drop", action="store_true", help="go silent 1 s into Z+ and hold the socket open")
    a = ap.parse_args()

    s = socket.create_connection((a.host, a.port), timeout=3)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    s.settimeout(None)                                 # the 3 s was for connecting only
    s.sendall(b"H 4 FAKE_phone\n")
    last = [""]

    def reader():
        for line in s.makefile("r", encoding="utf-8"):
            last[0] = line.strip()
    threading.Thread(target=reader, daemon=True).start()

    seq, t0, k = 0, time.perf_counter(), 0
    frame = "T" if a.tool else "B"
    for dur, ax in SCRIPT:
        for _ in range(int(dur * 50)):
            t = k / 50
            if a.drop and 1.5 <= t < 3.0:
                pass                                   # silent, connection still open
            else:
                zero = (0,) * 6
                ra = ax if a.arm in ("RA", "both") else zero
                la = ax if a.arm in ("LA", "both") else zero
                s.sendall((f"K {seq} {a.speed} {frame} " + " ".join(map(str, ra + la)) + "\n").encode())
                seq += 1
            k += 1
            sl = t0 + k / 50 - time.perf_counter()
            if sl > 0:
                time.sleep(sl)
        print(f"t={k/50:4.1f} after {ax}  PC: {last[0]}", flush=True)
    s.close()


if __name__ == "__main__":
    main()
