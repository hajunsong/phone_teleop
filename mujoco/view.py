"""Open the MuJoCo model in the interactive viewer.

    python view.py                       slider mode (default)
    python view.py --key assembly        slider mode, start from q = 0
    python view.py --torque              RecurDyn-faithful torque model, no torque
    python view.py --torque --hold       ... with closed-loop gravity compensation

Slider mode loads mjcf/scene_servo.xml: every joint has a position servo and
gravity is cancelled (gravcomp).  In the viewer's right-hand panel open
"Control": one slider per joint (RA_servo1..7, LA_servo1..7), value = target
angle in rad.  "Joint" shows the actual angles.  The q6 slider stops 10 deg short
of the wrist parallelogram's singular pose.  Keyframes ("Load key" under
Simulation) reset pose and sliders together.

Torque mode is the model verify.py checks; its Control sliders are torques [N*m].
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import mujoco as mj
import mujoco.viewer

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import loops  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--key", default="rmd_ic", choices=["rmd_ic", "assembly"])
    ap.add_argument("--torque", action="store_true", help="torque-motor model instead of servos")
    ap.add_argument("--hold", action="store_true", help="(torque mode) gravity compensation")
    a = ap.parse_args()

    scene = "scene.xml" if a.torque else "scene_servo.xml"
    m = mj.MjModel.from_xml_path(str(HERE / "mjcf" / scene))
    d = mj.MjData(m)
    mj.mj_resetDataKeyframe(m, d, m.key(a.key).id)       # qpos and, in servo mode, ctrl
    loops.close_loops(m, d)
    mj.mj_forward(m, d)

    if not a.torque:
        mujoco.viewer.launch(m, d)                         # sliders drive d.ctrl directly
        return 0

    act_dof = [m.jnt_dofadr[m.actuator_trnid[i, 0]] for i in range(m.nu)]
    scratch = mj.MjData(m)
    with mujoco.viewer.launch_passive(m, d) as v:
        while v.is_running():
            t0 = time.perf_counter()
            if a.hold:
                scratch.qpos[:] = d.qpos
                tau, _ = loops.loop_gravity_torque(m, scratch)
                d.ctrl[:] = tau[act_dof]
            mj.mj_step(m, d)
            v.sync()
            time.sleep(max(0.0, m.opt.timestep - (time.perf_counter() - t0)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
