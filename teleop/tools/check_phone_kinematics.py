"""Check the phone's Java kinematics (RobotModel.java) against MuJoCo on a PC.

    python tools/check_phone_kinematics.py

Compiles RobotModel.java + tools/java/KinCheck.java with the Android Studio JDK
and compares, for random joint vectors inside the exported limits:
  - every serial-chain body position and both tool poses  (FK)
  - DLS IK from the home pose to a reachable random target (convergence)
  - Controller.java jog behaviour (tools/java/ControllerCheck.java): reach from
    home in 12 directions, no off-axis drift, step/mirror/tool frame, no wind-up
    at limits, continuous joint-space home move
Wrist parallelogram bodies (passive) are compared with a looser tolerance:
the phone uses the linear fit printed by export_phone_model.py.
"""

from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path

import mujoco as mj
import numpy as np

HERE = Path(__file__).resolve().parent
TELEOP = HERE.parent
sys.path.insert(0, str(TELEOP.parent / "mujoco"))
import loops  # noqa: E402

JDK = Path(r"C:\Program Files\Android\Android Studio\jbr\bin")


def main() -> int:
    m = mj.MjModel.from_xml_path(str(TELEOP.parent / "mujoco" / "mjcf" / "scene_servo.xml"))
    d = mj.MjData(m)
    model_txt = TELEOP / "android" / "assets" / "robot_model.txt"
    lim = {}
    for line in model_txt.read_text().splitlines():
        t = line.split()
        if t and t[0] in ("qmin", "qmax"):
            lim[(t[0], t[1])] = np.array([float(v) for v in t[2:9]])

    out = Path(tempfile.mkdtemp())
    src = TELEOP / "android" / "src" / "com" / "humanoid" / "teleop"
    srcs = [src / "RobotModel.java", src / "Controller.java", HERE / "java" / "KinCheck.java",
            HERE / "java" / "ControllerCheck.java"]
    subprocess.run([str(JDK / "javac.exe"), "-encoding", "UTF-8", "-d", str(out), *map(str, srcs)], check=True)
    p = subprocess.Popen([str(JDK / "java.exe"), "-cp", str(out), "KinCheck", str(model_txt)],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)

    rng = np.random.default_rng(0)
    sides = ("RA", "LA")
    passive = {m.body(f"{s}_body6_{k}").id for s in sides for k in (1, 2)}
    errs, perrs, terr, rerr = [], [], [], []
    for _ in range(200):
        q = {s: rng.uniform(lim[("qmin", s)], lim[("qmax", s)]) for s in sides}
        p.stdin.write("F " + " ".join(f"{v:.17g}" for v in np.concatenate([q["RA"], q["LA"]])) + "\n")
        p.stdin.flush()
        v = np.array([float(x) for x in p.stdout.readline().split()[1:]])
        mj.mj_resetData(m, d)
        for s in sides:
            for k in range(7):
                d.qpos[m.joint(f"{s}_joint{k + 1}").qposadr[0]] = q[s][k]
        loops.close_loops(m, d)
        mj.mj_kinematics(m, d)
        bp = v[:3 * (m.nbody - 1)].reshape(-1, 3)
        for b in range(1, m.nbody):
            e = np.linalg.norm(bp[b - 1] - d.xpos[b])
            (perrs if b in passive else errs).append(e)
        tv = v[3 * (m.nbody - 1):].reshape(2, 12)
        for i, s in enumerate(sides):
            sid = m.site(f"{s}_tool").id
            terr.append(np.linalg.norm(tv[i, :3] - d.site_xpos[sid]))
            rerr.append(np.abs(tv[i, 3:] - d.site_xmat[sid]).max())
    print(f"FK  serial bodies   max {max(errs):.2e} m")
    print(f"FK  tool position   max {max(terr):.2e} m,  rotation max {max(rerr):.2e}")
    print(f"FK  passive bodies  max {max(perrs) * 1e3:.3f} mm  (linear fit, rendering only)")

    ok, ms, ep_all = 0, [], []
    for _ in range(100):
        for s in sides:
            lo, hi = lim[("qmin", s)], lim[("qmax", s)]
            qt = rng.uniform(lo + 0.3 * (hi - lo), hi - 0.3 * (hi - lo))      # reachable target
            mj.mj_resetData(m, d)
            for k in range(7):
                d.qpos[m.joint(f"{s}_joint{k + 1}").qposadr[0]] = qt[k]
            mj.mj_kinematics(m, d)
            sid = m.site(f"{s}_tool").id
            q0 = (lo + hi) / 2
            p.stdin.write(f"I {s} " + " ".join(f"{x:.17g}" for x in
                                               np.concatenate([q0, d.site_xpos[sid], d.site_xmat[sid]])) + "\n")
            p.stdin.flush()
            r = p.stdout.readline().split()
            ep, er, t = float(r[8]), float(r[9]), float(r[10])
            ok += ep < 1e-4 and er < 1e-3
            ms.append(t)
            ep_all.append(ep)
    print(f"IK  converged {ok}/200 (pos < 0.1 mm, rot < 0.06 deg) from mid-range start, "
          f"median {np.median(ms):.2f} ms (desktop JVM, 200 iters max)")
    p.stdin.close()
    p.wait()
    good = max(errs) < 1e-9 and max(terr) < 1e-9 and max(rerr) < 1e-9 and max(perrs) < 2e-3
    print("kinematics", "PASS" if good else "FAIL")
    print("controller:")
    r = subprocess.run([str(JDK / "java.exe"), "-Dstdout.encoding=UTF-8", "-cp", str(out),
                        "com.humanoid.teleop.ControllerCheck", str(model_txt)],
                       capture_output=True, text=True, encoding="utf-8")
    print(r.stdout.rstrip())
    good = good and r.returncode == 0
    print("PASS" if good else "FAIL")
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
