"""Standalone MQTT broker for the phone teleoperation on Windows.

    python broker.py                 :1883 on all interfaces, adb reverse for USB phones, UDP beacon
    python broker.py --no-wifi       127.0.0.1 only (USB phones via adb reverse)

Runs the same broker teleop_sim.py embeds, as its own process, so phones stay
connected while the simulation (or anything else on the topics) is restarted.
Use the simulation against it with

    python teleop_sim.py --broker 127.0.0.1 --publish-state --no-usb --no-wifi

(--publish-state because on this broker the simulation stands in for the robot
bridge; never use it on a broker where the real bridge publishes joint_state).
Logs every client connect / disconnect and, once a second, the message rate per topic.
"""

from __future__ import annotations

import argparse
import sys
import time

from mqtt_lite import Broker, Client
from net_tools import Beacon, UsbReverser, local_ipv4


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--beacon-port", type=int, default=5000)
    ap.add_argument("--no-usb", action="store_true")
    ap.add_argument("--no-wifi", action="store_true")
    ap.add_argument("--quiet", action="store_true", help="no per-second topic rates")
    a = ap.parse_args()

    b = Broker(("127.0.0.1" if a.no_wifi else "0.0.0.0", a.port))
    try:
        b.start()
    except OSError as exc:
        print(f"[mqtt] cannot listen on :{a.port} ({exc}) - another broker running?")
        return 2
    usb = beacon = None
    if not a.no_usb:
        usb = UsbReverser(a.port)
        usb.start()
    if not a.no_wifi:
        beacon = Beacon(a.port, a.beacon_port)
        beacon.start()
    print(f"[mqtt] broker :{a.port}" + ("" if a.no_wifi else
          f"   Wi-Fi: phone -> {' / '.join(local_ipv4()) or '?'}:{a.port}  (UDP beacon :{a.beacon_port})"),
          flush=True)

    counts: dict[str, int] = {}
    mon = None
    if not a.quiet:
        mon = Client("broker_monitor")
        mon.connect("127.0.0.1", a.port)
        mon.subscribe("/humanoid/#", lambda t, p: counts.__setitem__(t, counts.get(t, 0) + 1))
    try:
        while True:
            time.sleep(1.0)
            if counts:
                print("[rate] " + "  ".join(f"{t} {n}/s" for t, n in sorted(counts.items())), flush=True)
                counts.clear()
    except KeyboardInterrupt:
        pass
    finally:
        if mon:
            mon.close()
        b.stop()
        if usb:
            usb.stop()
        if beacon:
            beacon.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
