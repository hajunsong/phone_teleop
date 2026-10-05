"""MuJoCo stand-in for the upper-body robot on MQTT (simulation only).

    python teleop_sim.py                         embedded broker on :1883, USB + Wi-Fi, viewer
    python teleop_sim.py --broker 192.168.0.142  use an existing broker instead (e.g. the robot's)
    python teleop_sim.py --no-wifi               broker on 127.0.0.1 only (USB via adb reverse)
    python teleop_sim.py --headless --duration 10

The phone app solves the IK itself and publishes joint targets exactly as the
robot expects them; this program plays the robot's part:

    subscribe  /humanoid/upper/cmd/joint    [16 q (rad, LA first), servo, (Kp), (Kd)]
    publish    /humanoid/upper/joint_state  {"t", "n": 16, "pos_rad", "vel_rad_s", "tau_a", "tau_nm"}  100 Hz

Array layout (humanoid_control/Mqtt/TOPICS.md): index 0-1 unused (torso),
2-8 LA q1-q7, 9-15 RA q1-q7.  Commands are validated with the bridge's rules
(17/18/19 numbers, finite, servo flag 0/1, gains in (0, 1]); a bad one is
dropped and counted.  The servo flag is shown but not enforced: the model
follows in either case so the motion can be seen.  q is the rmd/controller q
(same joint axes and zeros), so no conversion is applied.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import mujoco as mj
import numpy as np

HERE = Path(__file__).resolve().parent
MUJOCO_DIR = HERE.parent / "mujoco"
sys.path.insert(0, str(MUJOCO_DIR))
import loops  # noqa: E402

from mqtt_lite import Broker, Client  # noqa: E402
from net_tools import Beacon, UsbReverser, local_ipv4  # noqa: E402

TOPIC_CMD = "/humanoid/upper/cmd/joint"
TOPIC_STATE = "/humanoid/upper/joint_state"
SLOTS = {"LA": range(2, 9), "RA": range(9, 16)}


class CommandBox:
    """Latest valid command, written by the MQTT thread, read by the sim loop."""

    def __init__(self):
        self.lock = threading.Lock()
        self.q = None                 # {"LA": 7, "RA": 7}
        self.servo = 0
        self.gains = None
        self.n_ok = 0
        self.n_bad = 0
        self.t_last = 0.0
        self.rate = 0.0
        self.max_step_deg = 0.0       # largest per-message joint change seen (jump check)
        self.last_bad = ""

    def on_message(self, topic: str, payload: bytes):
        try:
            v = json.loads(payload)
            if not isinstance(v, list) or len(v) not in (17, 18, 19):
                raise ValueError(f"{len(v) if isinstance(v, list) else type(v).__name__} values")
            v = [float(x) for x in v]
            if not all(math.isfinite(x) for x in v):
                raise ValueError("NaN/Inf")
            if v[16] not in (0.0, 1.0):
                raise ValueError("servo flag not 0/1")
            if any(not 0.0 < g <= 1.0 for g in v[17:]):
                raise ValueError("gain outside (0, 1]")
        except (ValueError, TypeError) as exc:
            with self.lock:
                self.n_bad += 1
                self.last_bad = str(exc)
            return
        q = {s: np.array([v[i] for i in SLOTS[s]]) for s in SLOTS}
        now = time.perf_counter()
        with self.lock:
            if self.q is not None:
                step = max(np.abs(q[s] - self.q[s]).max() for s in SLOTS)
                self.max_step_deg = max(self.max_step_deg, math.degrees(step))
            if self.t_last:
                self.rate += 0.05 * (1.0 / max(1e-3, now - self.t_last) - self.rate)
            self.q, self.servo, self.gains = q, int(v[16]), v[17:]
            self.n_ok += 1
            self.t_last = now


def draw_frame(scn, p, R, length=0.06, width=0.004):
    for k, rgba in enumerate(([1, 0.2, 0.2, 1], [0.2, 1, 0.2, 1], [0.3, 0.4, 1, 1])):
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mj.mjv_initGeom(g, mj.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.zeros(9),
                        np.array(rgba, np.float32))
        mj.mjv_connector(g, mj.mjtGeom.mjGEOM_ARROW, width, p, p + length * R[:, k])
        scn.ngeom += 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--broker", default="", help="HOST[:PORT] of an existing broker (default: run one here)")
    ap.add_argument("--mqtt-port", type=int, default=1883, help="port of the embedded broker")
    ap.add_argument("--beacon-port", type=int, default=5000)
    ap.add_argument("--no-usb", action="store_true", help="do not set up adb reverse")
    ap.add_argument("--no-wifi", action="store_true", help="embedded broker on 127.0.0.1 only, no beacon")
    ap.add_argument("--key", default="rmd_ic", choices=["rmd_ic", "assembly"])
    ap.add_argument("--state-hz", type=float, default=100.0)
    ap.add_argument("--publish-state", action="store_true",
                    help="publish joint_state even on an external --broker (never next to the real bridge)")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--duration", type=float, default=0.0, help="stop after N s (0 = until closed)")
    a = ap.parse_args()

    m = mj.MjModel.from_xml_path(str(MUJOCO_DIR / "mjcf" / "scene_servo.xml"))
    d = mj.MjData(m)
    mj.mj_resetDataKeyframe(m, d, m.key(a.key).id)
    loops.close_loops(m, d)
    mj.mj_forward(m, d)
    act = {s: [m.actuator(f"{s}_servo{i}").id for i in range(1, 8)] for s in SLOTS}
    qadr = {s: [m.jnt_qposadr[m.actuator_trnid[i, 0]] for i in act[s]] for s in SLOTS}
    dadr = {s: [m.jnt_dofadr[m.actuator_trnid[i, 0]] for i in act[s]] for s in SLOTS}
    tool = {s: m.site(f"{s}_tool").id for s in SLOTS}

    broker = None
    if a.broker:
        host, _, port = a.broker.partition(":")
        port = int(port or 1883)
    else:
        host, port = "127.0.0.1", a.mqtt_port
        broker = Broker(("127.0.0.1" if a.no_wifi else "0.0.0.0", port))
        try:
            broker.start()
        except OSError as exc:
            print(f"[mqtt] cannot listen on :{port} ({exc}). Another broker running? "
                  f"Use --broker 127.0.0.1:{port}")
            return 2
    cmd = CommandBox()
    cli = Client("teleop_sim")
    cli.connect(host, port)
    cli.subscribe(TOPIC_CMD, cmd.on_message)

    usb = beacon = None
    if not a.no_usb:
        usb = UsbReverser(port)
        usb.start()
    if not a.no_wifi:
        beacon = Beacon(port, a.beacon_port)
        beacon.start()
    where = f"{host}:{port}" if a.broker else (
        f"embedded :{port}" + ("" if a.no_wifi else f"   Wi-Fi: phone -> {' / '.join(local_ipv4()) or '?'}:{port}"
                               f"  (UDP beacon :{a.beacon_port})"))
    publish_state = not a.broker or a.publish_state
    if not publish_state:
        print("[mqtt] external broker: joint_state is NOT published (the real bridge owns it); "
              "the phone will hold until it hears one. --publish-state overrides.")
    print(f"[mqtt] broker {where}")
    print(f"[mqtt] subscribe {TOPIC_CMD}   publish {TOPIC_STATE} @ {a.state_hz:.0f} Hz", flush=True)

    viewer = None
    if not a.headless:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(m, d)

    dt_ctl = 0.01
    n_sub = max(1, round(dt_ctl / m.opt.timestep))
    t_start = time.perf_counter()
    t_next = t_start
    t_state = t_print = 0.0
    try:
        while True:
            now = time.perf_counter()
            if a.duration and now - t_start > a.duration:
                break
            if viewer is not None and not viewer.is_running():
                break

            with cmd.lock:
                q_cmd = None if cmd.q is None else {s: cmd.q[s].copy() for s in SLOTS}
            if q_cmd is not None:
                for s in SLOTS:
                    lo = m.actuator_ctrlrange[act[s], 0]
                    hi = m.actuator_ctrlrange[act[s], 1]
                    d.ctrl[act[s]] = np.clip(q_cmd[s], lo, hi)
            for _ in range(n_sub):
                mj.mj_step(m, d)

            if publish_state and now - t_state >= 1.0 / a.state_hz:
                t_state = now
                pos, vel = [0.0] * 16, [0.0] * 16
                for s in SLOTS:
                    for k, i in enumerate(SLOTS[s]):
                        pos[i] = float(d.qpos[qadr[s][k]])
                        vel[i] = float(d.qvel[dadr[s][k]])
                cli.publish(TOPIC_STATE, json.dumps({
                    "t": time.time_ns(), "n": 16,
                    "pos_rad": [round(x, 6) for x in pos], "vel_rad_s": [round(x, 6) for x in vel],
                    "tau_a": [0.0] * 16, "tau_nm": [0.0] * 16}).encode())

            if now - t_print > 1.0:
                t_print = now
                with cmd.lock:
                    age = now - cmd.t_last if cmd.t_last else float("inf")
                    info = (f"cmd {cmd.n_ok} ok / {cmd.n_bad} bad, {cmd.rate:.0f} Hz, "
                            f"age {age * 1e3:.0f} ms, servo {cmd.servo}, max step {cmd.max_step_deg:.2f} deg")
                    if cmd.n_bad and cmd.last_bad:
                        info += f", last bad: {cmd.last_bad}"
                tp = "  ".join(f"{s}=({d.site_xpos[tool[s]][0]:+.3f},{d.site_xpos[tool[s]][1]:+.3f},"
                               f"{d.site_xpos[tool[s]][2]:+.3f})" for s in ("LA", "RA"))
                print(f"[sim] {info}  {tp}", flush=True)

            if viewer is not None:
                with viewer.lock():
                    viewer.user_scn.ngeom = 0
                    for s in SLOTS:
                        draw_frame(viewer.user_scn, d.site_xpos[tool[s]], d.site_xmat[tool[s]].reshape(3, 3))
                viewer.sync()

            t_next += dt_ctl
            sl = t_next - time.perf_counter()
            if sl > 0:
                time.sleep(sl)
            else:
                t_next = time.perf_counter()
    except KeyboardInterrupt:
        pass
    finally:
        cli.close()
        if broker:
            broker.stop()
        if usb:
            usb.stop()
        if beacon:
            beacon.stop()
        if viewer is not None:
            viewer.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
