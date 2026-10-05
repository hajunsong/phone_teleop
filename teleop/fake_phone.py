"""Stand-in for the phone app on MQTT, for testing the PC side without a phone.

    python fake_phone.py                       broker 127.0.0.1:1883
    python fake_phone.py --host 192.168.0.10

Does what the app does at connect: subscribe joint_state, take the current
pose from it, then publish /humanoid/upper/cmd/joint at 50 Hz.  The script
swings LA q4 by +20 deg and RA q1 by -15 deg along a smooth (C1) profile and
back, then sends one malformed command to check the validator.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

from mqtt_lite import Client

CMD, STATE = "/humanoid/upper/cmd/joint", "/humanoid/upper/joint_state"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--seconds", type=float, default=4.0)
    a = ap.parse_args()

    state = {}
    c = Client("fake_phone")
    c.connect(a.host, a.port)
    c.subscribe(STATE, lambda t, p: state.update(json.loads(p)))
    t0 = time.time()
    while "pos_rad" not in state and time.time() - t0 < 2.0:
        time.sleep(0.02)
    if "pos_rad" not in state:
        print("no joint_state - is teleop_sim.py running?")
        return 1
    q0 = list(state["pos_rad"])
    print("start pose LA q4 %.2f deg, RA q1 %.2f deg" % (math.degrees(q0[5]), math.degrees(q0[9])))

    n = int(a.seconds * 50)
    for k in range(n + 1):
        s = math.sin(math.pi * k / n)                 # 0 -> 1 -> 0, smooth
        q = list(q0)
        q[5] += math.radians(20) * s                 # LA q4
        q[9] -= math.radians(15) * s                 # RA q1
        c.publish(CMD, json.dumps([round(v, 6) for v in q] + [0]).encode())
        if k == n // 2:
            time.sleep(0.3)
            p = state["pos_rad"]
            print("mid  pose LA q4 %.2f deg (cmd %.2f), RA q1 %.2f deg (cmd %.2f)" % (
                math.degrees(p[5]), math.degrees(q[5]), math.degrees(p[9]), math.degrees(q[9])))
        time.sleep(0.02)
    c.publish(CMD, b"[1,2,3]")                       # must be rejected
    time.sleep(0.5)
    p = state["pos_rad"]
    print("end  pose LA q4 %.2f deg, RA q1 %.2f deg" % (math.degrees(p[5]), math.degrees(p[9])))
    c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
