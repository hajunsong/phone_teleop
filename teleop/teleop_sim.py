"""Phone jog-pad teleoperation of the MuJoCo upper-body model (simulation only).

    python teleop_sim.py                    USB + Wi-Fi, viewer
    python teleop_sim.py --no-wifi          USB only (server bound to 127.0.0.1)
    python teleop_sim.py --no-usb           Wi-Fi only (no adb reverse)
    python teleop_sim.py --headless --duration 10      no viewer (tests)

The phone app shows two jog pads in landscape (left half LA, right half RA;
X/Y/Z, Rx/Ry/Rz, each -/+), no motion sensors.  One phone drives both arms.
Pipeline per control tick (100 Hz):

    held buttons (-1/0/+1 per axis), speed level, frame
      --integrate at a fixed speed-->  tool target pose of that arm
      --DLS IK on the MuJoCo model-->  q_cmd (7)
      --> position-servo setpoints of scene_servo.xml (gravity compensated)

Jog frame: base (x forward, y left, z up; rotations about base axes through
the tool point) or tool (axes of the tool site).  Releasing every button stops
the target; so does a gap of more than --watchdog s in the phone's 50 Hz
stream (a dropped link cannot leave the arm moving).
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import mujoco as mj
import numpy as np

HERE = Path(__file__).resolve().parent
MUJOCO_DIR = HERE.parent / "mujoco"
sys.path.insert(0, str(MUJOCO_DIR))
import loops  # noqa: E402

from phone_server import Beacon, PhoneServer, UsbReverser, local_ipv4  # noqa: E402

ARMS = ("RA", "LA")


def rotvec(R: np.ndarray) -> np.ndarray:
    """log map SO(3) -> axis*angle."""
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    th = math.acos(c)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if th < 1e-6:
        return 0.5 * v
    if th > math.pi - 1e-4:                             # near pi: use the symmetric part
        A = (R + np.eye(3)) / 2
        k = int(np.argmax(np.diag(A)))
        axis = A[:, k] / math.sqrt(max(A[k, k], 1e-12))
        return axis * th
    return th / (2 * math.sin(th)) * v


def expmap(w: np.ndarray) -> np.ndarray:
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


class ArmIk:
    """Damped least squares on the 7 actuated joints of one arm."""

    def __init__(self, m: mj.MjModel, side: str):
        self.m = m
        self.side = side
        self.d = mj.MjData(m)                           # scratch, kinematics only
        self.site = m.site(f"{side}_tool").id
        self.act = [m.actuator(f"{side}_servo{i}").id for i in range(1, 8)]
        jids = [m.actuator_trnid[a, 0] for a in self.act]
        self.qadr = np.array([m.jnt_qposadr[j] for j in jids])
        self.dadr = np.array([m.jnt_dofadr[j] for j in jids])
        self.lo = m.actuator_ctrlrange[self.act, 0].copy()
        self.hi = m.actuator_ctrlrange[self.act, 1].copy()
        self.jp = np.zeros((3, m.nv))
        self.jr = np.zeros((3, m.nv))

    def fk(self, q: np.ndarray):
        self.d.qpos[self.qadr] = q
        mj.mj_kinematics(self.m, self.d)
        return self.d.site_xpos[self.site].copy(), self.d.site_xmat[self.site].reshape(3, 3).copy()

    def solve(self, q0, p_des, R_des, iters=15, w_rot=0.2, lam=0.02, dq_max=0.2):
        """Returns (q, pos_err [m], rot_err [rad]).  w_rot [m/rad] weighs rotation
        against position; lam is the damping; dq_max caps each step."""
        q = np.array(q0, float)
        for _ in range(iters):
            p, R = self.fk(q)
            e_p = p_des - p
            e_r = rotvec(R_des @ R.T)
            if np.linalg.norm(e_p) < 1e-5 and np.linalg.norm(e_r) < 1e-4:
                break
            mj.mj_comPos(self.m, self.d)
            mj.mj_jacSite(self.m, self.d, self.jp, self.jr, self.site)
            J = np.vstack([self.jp[:, self.dadr], w_rot * self.jr[:, self.dadr]])
            e = np.concatenate([e_p, w_rot * e_r])
            dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e)
            n = np.linalg.norm(dq)
            if n > dq_max:
                dq *= dq_max / n
            q = np.clip(q + dq, self.lo, self.hi)
        p, R = self.fk(q)
        return q, float(np.linalg.norm(p_des - p)), float(np.linalg.norm(rotvec(R_des @ R.T)))


class ArmJog:
    """Jog integration + IK for one arm."""

    def __init__(self, ik: ArmIk, q_home: np.ndarray, a):
        self.ik = ik
        self.a = a
        self.q_home = q_home.copy()
        self.home()

    def home(self):
        self.q_cmd = self.q_home.copy()
        self.p_t, self.R_t = self.ik.fk(self.q_cmd)      # tool target
        self.moving = False
        self.err = (0.0, 0.0)
        self.state = "대기"

    def update(self, ph, dt: float, now: float):
        a = self.a
        side = self.ik.side
        if ph is None:
            self.state = "폰 없음"
            self._stop()
            return
        if now - ph.t_rx > a.watchdog:
            self.state = "신호 끊김 - 정지"
            self._stop()
            return
        jog = ph.jog[side]
        if not jog.any():
            self.state = "대기"
            self._stop()
            return

        lin = a.lin_speed[min(ph.speed, 2)]
        ang = math.radians(a.ang_speed[min(ph.speed, 2)])
        v = lin * jog[:3]
        w = ang * jog[3:]
        if ph.tool_frame:                               # tool axes -> world
            v = self.R_t @ v
            w = self.R_t @ w
        self.p_t = self.p_t + v * dt
        self.R_t = expmap(w * dt) @ self.R_t
        self.moving = True

        q, ep, er = self.ik.solve(self.q_cmd, self.p_t, self.R_t, dq_max=a.max_joint_speed * dt)
        self.q_cmd = q
        self.err = (ep, er)
        if ep > a.reach_tol or er > math.radians(a.reach_tol_deg):
            # out of reach / joint limit: the target stays on what the arm can do,
            # so backing off responds at once instead of first unwinding an overshoot
            self.p_t, self.R_t = self.ik.fk(self.q_cmd)
            self.state = "한계 도달"
        else:
            self.state = "이동 중"

    def _stop(self):
        if self.moving:
            self.p_t, self.R_t = self.ik.fk(self.q_cmd)  # hold exactly where the arm is
        self.moving = False


def draw_frame(scn, p, R, length=0.06, width=0.004, alpha=1.0):
    for k, rgba in enumerate(([1, 0.2, 0.2, alpha], [0.2, 1, 0.2, alpha], [0.3, 0.4, 1, alpha])):
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mj.mjv_initGeom(g, mj.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.zeros(9),
                        np.array(rgba, np.float32))
        mj.mjv_connector(g, mj.mjtGeom.mjGEOM_ARROW, width, p, p + length * R[:, k])
        scn.ngeom += 1


def triple(s: str) -> list[float]:
    v = [float(x) for x in s.split(",")]
    if len(v) != 3:
        raise argparse.ArgumentTypeError("need three comma-separated values: slow,normal,fast")
    return v


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=5001)
    ap.add_argument("--beacon-port", type=int, default=5000)
    ap.add_argument("--no-usb", action="store_true", help="do not set up adb reverse")
    ap.add_argument("--no-wifi", action="store_true", help="listen on 127.0.0.1 only, no beacon")
    ap.add_argument("--key", default="rmd_ic", choices=["rmd_ic", "assembly"])
    ap.add_argument("--lin-speed", type=triple, default=[0.02, 0.05, 0.10],
                    help="jog speed [m/s] for slow,normal,fast (default 0.02,0.05,0.10)")
    ap.add_argument("--ang-speed", type=triple, default=[10.0, 25.0, 45.0],
                    help="jog rate [deg/s] for slow,normal,fast (default 10,25,45)")
    ap.add_argument("--max-joint-speed", type=float, default=3.0, help="IK joint step limit [rad/s]")
    ap.add_argument("--reach-tol", type=float, default=0.005, help="IK error that counts as out of reach [m]")
    ap.add_argument("--reach-tol-deg", type=float, default=3.0)
    ap.add_argument("--watchdog", type=float, default=0.2, help="stop if no button state for this long [s]")
    ap.add_argument("--rate", type=float, default=100.0, help="control rate [Hz]")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--duration", type=float, default=0.0, help="stop after N s (0 = until closed)")
    a = ap.parse_args()

    m = mj.MjModel.from_xml_path(str(MUJOCO_DIR / "mjcf" / "scene_servo.xml"))
    d = mj.MjData(m)
    mj.mj_resetDataKeyframe(m, d, m.key(a.key).id)
    loops.close_loops(m, d)
    mj.mj_forward(m, d)

    arms = {}
    for side in ARMS:
        ik = ArmIk(m, side)
        arms[side] = ArmJog(ik, d.ctrl[ik.act].copy(), a)

    srv = PhoneServer(a.port, "127.0.0.1" if a.no_wifi else "0.0.0.0")
    srv.start()
    usb = beacon = None
    if not a.no_usb:
        usb = UsbReverser(a.port)
        usb.start()
    if not a.no_wifi:
        beacon = Beacon(a.port, a.beacon_port)
        beacon.start()
    print(f"[net] TCP {('127.0.0.1' if a.no_wifi else '0.0.0.0')}:{a.port}"
          + ("" if a.no_wifi else f"   Wi-Fi: phone -> {' / '.join(local_ipv4()) or '?'}:{a.port}"
             f"  (UDP beacon :{a.beacon_port})"))
    print("[ui ] 폰의 조그 버튼을 누르고 있는 동안만 팔 끝단이 움직입니다. 떼면 정지.", flush=True)

    dt_ctl = 1.0 / a.rate
    n_sub = max(1, round(dt_ctl / m.opt.timestep))
    viewer = None
    if not a.headless:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(m, d)

    t_start = time.perf_counter()
    t_next = t_start
    t_status = 0.0
    t_print = 0.0
    try:
        while True:
            now = time.perf_counter()
            if a.duration and now - t_start > a.duration:
                break
            if viewer is not None and not viewer.is_running():
                break

            phones = srv.snapshot()
            # the newest phone that has sent jog state drives both arms
            live = [cid for cid in sorted(phones) if phones[cid].t_rx]
            owner = live[-1] if live else None
            for cid, ph in phones.items():
                for ev, side in ph.events:
                    if ev == "HOME":
                        for s_ in (arms if side == "ALL" else [side]):
                            if s_ in arms:
                                arms[s_].home()
                        print(f"[ctl] {side} -> home", flush=True)
            for side, arm in arms.items():
                arm.update(phones.get(owner) if owner is not None else None, dt_ctl, now)
                d.ctrl[arm.ik.act] = arm.q_cmd

            for _ in range(n_sub):
                mj.mj_step(m, d)

            if now - t_status > 0.1:
                t_status = now
                parts = []
                for side in ("LA", "RA"):
                    arm = arms[side]
                    p = arm.p_t * 1e3
                    parts.append(f"{side} {arm.state} ({p[0]:.0f},{p[1]:.0f},{p[2]:.0f})mm")
                msg = "  |  ".join(parts)
                for cid in phones:
                    note = "" if cid == owner else "[다른 폰이 제어 중]  "
                    srv.send_status(cid, note + msg)
            if now - t_print > 1.0:
                t_print = now
                parts = []
                for side, arm in arms.items():
                    p = arm.p_t
                    parts.append(f"{side}:{arm.state} tool=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f})")
                print(f"[ctl] phones={len(phones)}  " + "  ".join(parts), flush=True)

            if viewer is not None:
                with viewer.lock():
                    viewer.user_scn.ngeom = 0
                    for arm in arms.values():
                        draw_frame(viewer.user_scn, arm.p_t, arm.R_t, alpha=1.0 if arm.moving else 0.35)
                viewer.sync()

            t_next += dt_ctl
            sl = t_next - time.perf_counter()
            if sl > 0:
                time.sleep(sl)
            else:
                t_next = time.perf_counter()        # fell behind: do not try to catch up
    except KeyboardInterrupt:
        pass
    finally:
        srv.stop()
        if usb:
            usb.stop()
        if beacon:
            beacon.stop()
        if viewer is not None:
            viewer.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
