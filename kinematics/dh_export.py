"""Modified DH (Craig) parameters of both arms, from the current HumanoidUpperBody.rmd.

    python dh_export.py            -> dh_params.json, dh_params.md (table), FK check printed

The rmd is turned into a URDF in a temporary folder by ../mujoco/rmd_to_urdf.py
(nothing in ../mujoco is touched), and the DH frames are built from the joint
axis LINES, not read from markers: z_i = joint axis i, x_i along the common
normal of axes i and i+1 (its sign chosen so alpha matches the RightArm template
below), origin where that normal meets axis i; x_7 points at the tool.  Frame 0
is the arm subsystem frame (body0).  On the RightArm this reproduces the rmd's
own Ai/Cij markers exactly (they were edited to be DH frames); the LeftArm
markers are not DH frames, so this construction is what gives its table.

Convention:  T_0^tool = prod_i  RotX(alpha_{i-1}) TransX(a_{i-1}) RotZ(theta_i) TransZ(d_i),
             theta_i = q_i + theta_off_i,   q_i = rmd joint angle (I marker about J marker z).
Every table is checked by forward kinematics against the model for random q.
"""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path

import mujoco as mj
import numpy as np

HERE = Path(__file__).resolve().parent
CLAUDE_WS = HERE.parent
TEMPLATE_ALPHA = [0, 90, 90, 90, 90, 90, -90]          # RightArm alpha_{i-1}, i = 1..7


def T(R, p):
    t = np.eye(4)
    t[:3, :3] = R
    t[:3, 3] = p
    return t


def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1.]])


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.]])


def Dx(a):
    t = np.eye(4)
    t[0, 3] = a
    return t


def Dz(a):
    t = np.eye(4)
    t[2, 3] = a
    return t


def dh_of(Tr):
    R, p = Tr[:3, :3], Tr[:3, 3]
    al = np.arctan2(-R[1, 2], R[2, 2])
    th = np.arctan2(-R[0, 1], R[0, 0])
    d = -np.sin(al) * p[1] + np.cos(al) * p[2]
    return al, p[0], d, th, float(np.abs(Rx(al) @ Dx(p[0]) @ Rz(th) @ Dz(d) - Tr).max())


def main() -> int:
    tmp = Path(tempfile.mkdtemp())
    subprocess.run([sys.executable, str(CLAUDE_WS / "mujoco" / "rmd_to_urdf.py"), "--out", str(tmp)],
                   check=True, stdout=subprocess.DEVNULL)
    m = mj.MjModel.from_xml_path(str(tmp / "urdf" / "humanoid_upper.urdf"))
    d = mj.MjData(m)
    out = {"convention": "Modified DH (Craig). T = RotX(alpha_{i-1}) TransX(a_{i-1}) RotZ(q_i + theta_off_i) TransZ(d_i). "
                         "mm, deg. Frame 0 = arm subsystem frame (body0), given in the robot base frame as T_base_0.",
           "source": "HumanoidUpperBody.rmd (via mujoco/rmd_to_urdf.py)"}
    for side in ("RA", "LA"):
        mj.mj_resetData(m, d)
        mj.mj_kinematics(m, d)
        b0 = m.body(f"{side}_body0").id
        F0 = T(d.xmat[b0].reshape(3, 3), d.xpos[b0])
        jb = [m.joint(f"{side}_joint{k}").bodyid[0] for k in range(1, 8)]
        z = [d.xmat[b].reshape(3, 3)[:, 2].copy() for b in jb]
        o = [d.xpos[b].copy() for b in jb]
        tool = [b for b in range(m.nbody) if m.body(b).name == f"{side}_tool"][0]
        pt, Rt = d.xpos[tool].copy(), d.xmat[tool].reshape(3, 3).copy()
        frames = [F0]
        for i in range(7):
            zi = z[i]
            if i < 6:
                zn = z[i + 1]
                c = np.cross(zi, zn)
                if np.linalg.norm(c) < 1e-9:
                    raise SystemExit(f"{side}: axes {i + 1} and {i + 2} are parallel - not handled")
                A = np.array([[zi @ zi, -zi @ zn], [zi @ zn, -zn @ zn]])
                b = np.array([(o[i + 1] - o[i]) @ zi, (o[i + 1] - o[i]) @ zn])
                t, _ = np.linalg.solve(A, b)
                org = o[i] + t * zi
                x = c / np.linalg.norm(c)
                if np.sign(np.arctan2(np.cross(zi, zn) @ x, zi @ zn)) != np.sign(TEMPLATE_ALPHA[i + 1]):
                    x = -x
            else:
                org = o[i] + ((pt - o[i]) @ zi) * zi
                x = (pt - org) / np.linalg.norm(pt - org)
            frames.append(T(np.column_stack([x, np.cross(zi, x), zi]), org))
        frames.append(T(Rt, pt))
        rows = [dh_of(np.linalg.inv(frames[i - 1]) @ frames[i]) for i in range(1, len(frames))]

        rng = np.random.default_rng(0)
        err = 0.0
        for _ in range(500):
            q = rng.uniform(-np.pi, np.pi, 7)
            for k in range(7):
                d.qpos[m.joint(f"{side}_joint{k + 1}").qposadr[0]] = q[k]
            mj.mj_kinematics(m, d)
            Tc = F0.copy()
            for i, (al, a, dd, th, _) in enumerate(rows):
                Tc = Tc @ Rx(al) @ Dx(a) @ Rz(th + (q[i] if i < 7 else 0)) @ Dz(dd)
            err = max(err, float(np.abs(Tc - T(d.xmat[tool].reshape(3, 3), d.xpos[tool])).max()))
        axes0 = [np.round(F0[:3, :3].T @ zz, 4).tolist() for zz in z]
        out[side] = {
            "rows": [{"i": (i + 1 if i < 7 else "tool"), "alpha_deg": round(float(np.degrees(r[0])), 6) + 0.0,
                      "a_mm": round(float(r[1] * 1e3), 6) + 0.0, "d_mm": round(float(r[2] * 1e3), 6) + 0.0,
                      "theta_off_deg": round(float(np.degrees(r[3])), 6) + 0.0} for i, r in enumerate(rows)],
            "T_base_0": {"p_mm": np.round(F0[:3, 3] * 1e3, 4).tolist(), "R": np.round(F0[:3, :3], 6).tolist()},
            "joint_axes_base_q0": [np.round(zz, 4).tolist() for zz in z],
            "joint_axes_frame0_q0": axes0,
            "fk_check_max_abs_err": err,
        }
        print(f"{side}: FK check over 500 random q: max |T_DH - T_model| = {err:.1e}")

    def fmt(v):
        return f"{v:g}" if abs(v) > 1e-9 else "0"

    md = []
    for side in ("RA", "LA"):
        md.append(f"| i | α(i−1) [°] | a(i−1) [mm] | d(i) [mm] | θ_off(i) [°] |   ({side})")
        md.append("|---|---|---|---|---|")
        for r in out[side]["rows"]:
            md.append(f"| {r['i']} | {fmt(r['alpha_deg'])} | {fmt(r['a_mm'])} | {fmt(r['d_mm'])} | {fmt(r['theta_off_deg'])} |")
        md.append("")
    (HERE / "dh_params.md").write_text("\n".join(md), encoding="utf-8")
    (HERE / "dh_params.json").write_text(json.dumps(out, indent=1, ensure_ascii=False), encoding="utf-8")
    print("\n".join(md))
    print(f"wrote {HERE / 'dh_params.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
