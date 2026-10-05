"""Verify the MuJoCo model against the verified MATLAB/C++ chain and the rmd.

A. Tree, against params/ref_{RA,LA}.csv (60 states each, MATLAB dump_reference)
   MATLAB folds the wrist linkage bodies onto their hosts (body6_1 -> body6,
   body6_2 -> body5) at the assembly pose.  An in-memory copy of the MuJoCo
   model does the same fold, with the loop constraint off, so the comparison is
   like for like.  It checks the URDF frames, joint axes/signs, masses, CoMs and
   inertia sign convention all at once:
     tool pose (FK), gravity torque, inverse dynamics (RNEA), mass matrix.

B. Closed loop, the model as shipped
   B1 loop closure: passive angles solved to the constraint, residual
   B2 gravity torque with the real parallel linkage vs the folded one
      (params/gravity_check_poses.csv).  The README's RecurDyn run left a
      0.246 N*mm residual on q6 against the folded torque; the same number
      should appear here as the loop-vs-fold difference at the rmd IC pose.
   B3 released from the rmd IC pose for 0.5 s: with the B2 torque held
      constant, and with no torque.  Tip sag, loop drift, energy.

Usage:  python verify.py            exit code 0 = all pass
"""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import mujoco as mj
import numpy as np

from loops import ARMS, close_loops, loop_gravity_torque, loop_residual, passive_adr  # noqa: F401

HERE = Path(__file__).resolve().parent
PARAMS = HERE.parent / "params"
MJCF = HERE / "mjcf" / "humanoid_upper.xml"
SIDECAR = json.loads((HERE / "urdf" / "humanoid_upper.loops.json").read_text(encoding="utf-8"))

results: list[tuple[str, float, float, bool]] = []


def check(label: str, value: float, tol: float) -> None:
    ok = bool(value <= tol)
    results.append((label, value, tol, ok))
    print(f"  [{'PASS' if ok else 'FAIL'}] {label:58s} {value:10.3e}  (tol {tol:.0e})")


def dofs(m, side):
    return np.array([m.jnt_dofadr[m.joint(f"{side}_joint{i}").id] for i in range(1, 8)])


def qadr(m, side):
    return np.array([m.jnt_qposadr[m.joint(f"{side}_joint{i}").id] for i in range(1, 8)])


def loop_bodies(side):
    return [b for b in (f"{side}_body6_1", f"{side}_body6_2")]


# ======================================================================
# A. folded tree vs MATLAB
# ======================================================================
def folded_model() -> mj.MjModel:
    m = mj.MjModel.from_xml_path(str(MJCF))
    d = mj.MjData(m)
    mj.mj_kinematics(m, d)                       # qpos0 = assembly
    mj.mj_comPos(m, d)
    m.eq_active0[:] = 0

    for side in ARMS:
        arm = json.loads((PARAMS / f"arm_{side}.json").read_text(encoding="utf-8"))
        hosts = {e: f"{side}_{L['name']}" for L in arm["links"]
                 for e in ([L["lumped"]] if isinstance(L["lumped"], str) else L["lumped"]) if e}
        for e_short, host in hosts.items():
            e, h = m.body(f"{side}_{e_short}").id, m.body(host).id
            Rh, ph = d.xmat[h].reshape(3, 3), d.xpos[h]

            def in_host(b, I_link=None):
                Ri = d.ximat[b].reshape(3, 3)
                c = Rh.T @ (d.xipos[b] - ph)
                if I_link is None:
                    I = Ri @ np.diag(m.body_inertia[b]) @ Ri.T
                else:                             # rmd original, given in the link frame
                    Rl = d.xmat[b].reshape(3, 3)
                    I = Rl @ np.asarray(I_link) @ Rl.T
                return m.body_mass[b], c, Rh.T @ I @ Rh

            rep = SIDECAR["inertial"][m.body(e).name].get("repaired")
            m1, c1, I1 = in_host(h)
            m2, c2, I2 = in_host(e, rep["inertia_rmd"] if rep else None)
            mt = m1 + m2
            ct = (m1 * c1 + m2 * c2) / mt
            It = sum(I + mm * ((c - ct) @ (c - ct) * np.eye(3) - np.outer(c - ct, c - ct))
                     for mm, c, I in ((m1, c1, I1), (m2, c2, I2)))
            w, V = np.linalg.eigh(It)
            if np.linalg.det(V) < 0:
                V[:, 0] *= -1
            q = np.zeros(4)
            mj.mju_mat2Quat(q, V.ravel())
            m.body_mass[h], m.body_ipos[h], m.body_inertia[h], m.body_iquat[h] = mt, ct, w, q
            m.body_mass[e], m.body_inertia[e] = 1e-15, 1e-18
            m.dof_armature[m.jnt_dofadr[m.body_jntadr[e]]] = 1.0   # keep M invertible
    return m


def part_a():
    print("\nA. folded tree vs MATLAB reference (params/ref_*.csv)")
    m = folded_model()
    d = mj.MjData(m)
    for side in ARMS:
        ref = np.loadtxt(PARAMS / f"ref_{side}.csv", delimiter=",")
        dv, qa = dofs(m, side), qadr(m, side)
        site = m.site(f"{side}_tool").id
        e = dict(p=0, R=0, g=0, rnea=0, M=0)
        for row in ref:
            q, qd, qdd = row[0:7], row[7:14], row[14:21]
            d.qpos[:] = 0; d.qvel[:] = 0; d.qacc[:] = 0
            d.qpos[qa], d.qvel[dv] = q, 0
            mj.mj_forward(m, d)
            e["p"] = max(e["p"], np.abs(d.site_xpos[site] - row[44:47]).max())
            e["R"] = max(e["R"], np.abs(d.site_xmat[site] - row[35:44]).max())
            e["g"] = max(e["g"], np.abs(d.qfrc_bias[dv] - row[28:35]).max())
            M = np.zeros((m.nv, m.nv))
            mj.mj_fullM(m, d, M)
            e["M"] = max(e["M"], np.abs(M[np.ix_(dv, dv)] - row[47:96].reshape(7, 7)).max())
            d.qvel[dv], d.qacc[dv] = qd, qdd
            mj.mj_forward(m, d)
            d.qacc[:] = 0; d.qacc[dv] = qdd
            tau = np.zeros(m.nv)
            mj.mj_rne(m, d, 1, tau)
            e["rnea"] = max(e["rnea"], np.abs(tau[dv] - row[21:28]).max())
        check(f"{side} tool position       [m]", e["p"], 1e-12)
        check(f"{side} tool orientation    [-]", e["R"], 1e-12)
        check(f"{side} gravity torque      [N*m]", e["g"], 1e-10)
        check(f"{side} inverse dynamics    [N*m]", e["rnea"], 1e-9)
        check(f"{side} mass matrix         [kg*m^2]", e["M"], 1e-11)


# ======================================================================
# B. closed loop
# ======================================================================
def part_b():
    print("\nB. closed-loop model")
    m = mj.MjModel.from_xml_path(str(MJCF))
    mf = folded_model()
    d, df = mj.MjData(m), mj.MjData(mf)

    # B1 + B2 over the MATLAB gravity poses
    rows = list(csv.DictReader(open(PARAMS / "gravity_check_poses.csv", encoding="utf-8")))
    worst_res, worst_pas = 0.0, 0.0
    diff = {s: np.zeros(7) for s in ARMS}
    print("   loop-vs-fold gravity torque difference [N*mm] per pose:")
    for r in rows:
        s = r["side"]
        q = np.array([float(r[f"q{i}_rad"]) for i in range(1, 8)])
        tau_ml = np.array([float(r[f"tau{i}_Nm"]) for i in range(1, 8)])
        mj.mj_resetData(m, d)
        d.qpos[qadr(m, s)] = q
        worst_res = max(worst_res, close_loops(m, d))
        tau, pas = loop_gravity_torque(m, d)
        worst_pas = max(worst_pas, pas)
        dt = (tau[dofs(m, s)] - tau_ml) * 1e3
        diff[s] = np.maximum(diff[s], np.abs(dt))
        print(f"     {s} {r['pose']:>22s}  " + " ".join(f"{x:+8.3f}" for x in dt))
    check("B1 loop closure residual after solve   [m]", worst_res, 1e-12)
    check("B2 passive-joint torque left over       [N*m]", worst_pas, 1e-12)
    for s in ARMS:
        print(f"   {s} max |loop - fold| per joint [N*mm]: " + " ".join(f"{x:.3f}" for x in diff[s]))

    # the rmd IC pose: compare with README (RecurDyn residual on q6 = 0.246 N*mm)
    mj.mj_resetDataKeyframe(m, d, m.key("rmd_ic").id)
    close_loops(m, d)
    tau_loop, _ = loop_gravity_torque(m, d)
    mj.mj_resetDataKeyframe(mf, df, mf.key("rmd_ic").id)
    mj.mj_forward(mf, df)
    print("   rmd IC pose, loop - fold [N*mm]:")
    for s in ARMS:
        dv = dofs(m, s)
        print(f"     {s} " + " ".join(f"{x:+.3f}" for x in (tau_loop[dv] - df.qfrc_bias[dofs(mf, s)]) * 1e3))

    # B3 release from the IC pose
    print("   release from rmd IC pose, 0.5 s:")
    m.opt.enableflags |= mj.mjtEnableBit.mjENBL_ENERGY
    act_dof = np.array([m.actuator_trnid[a, 0] for a in range(m.nu)])
    act_dof = np.array([m.jnt_dofadr[j] for j in act_dof])
    sites = [m.site(f"{s}_tool").id for s in ARMS]
    out = {}
    for label, ctrl in (("hold (loop gravity torque)", tau_loop[act_dof]),
                        ("no torque", np.zeros(m.nu))):
        mj.mj_resetDataKeyframe(m, d, m.key("rmd_ic").id)
        close_loops(m, d)
        mj.mj_forward(m, d)
        z0 = d.site_xpos[sites, 2].copy()
        E0 = d.energy.sum()
        d.ctrl[:] = ctrl
        loop_max = 0.0
        while d.time < 0.5 - 1e-12:
            mj.mj_step(m, d)
            loop_max = max(loop_max, np.abs(loop_residual(m, d)).max())
        sag = (z0 - d.site_xpos[sites, 2]) * 1e3
        out[label] = (sag, loop_max, d.energy.sum() - E0)
        print(f"     {label:28s} sag RA {sag[0]:8.3f} mm  LA {sag[1]:8.3f} mm   "
              f"loop drift max {loop_max * 1e6:.3f} um   dE {d.energy.sum() - E0:+.4f} J")
    check("B3 hold: tip sag after 0.5 s            [mm]", out["hold (loop gravity torque)"][0].max(), 1.0)
    # MuJoCo's loop closure is a soft constraint (RecurDyn's is hard, TOLERANCEIF 1e-8).
    # Peak drift during the violent free fall is ~40 um on an 80 mm link; stiffer
    # solimp/solref at dt = 0.5 ms go unstable (see README), so 0.1 mm is the bound.
    check("B3 loop drift, free fall                [m]", out["no torque"][1], 1e-4)
    check("B3 loop drift, hold                     [m]", out["hold (loop gravity torque)"][1], 1e-6)


# ======================================================================
# C. servo variant (viewer sliders)
# ======================================================================
def part_c():
    print("\nC. servo variant (mjcf/humanoid_upper_servo.xml, viewer sliders)")
    m = mj.MjModel.from_xml_path(str(HERE / "mjcf" / "humanoid_upper_servo.xml"))
    d = mj.MjData(m)
    qa = np.array([m.jnt_qposadr[m.actuator_trnid[i, 0]] for i in range(m.nu)])

    def run(T):
        worst = 0.0
        while d.time < T - 1e-12:
            mj.mj_step(m, d)
            worst = max(worst, np.abs(loop_residual(m, d)).max())
        return worst

    mj.mj_resetDataKeyframe(m, d, m.key("rmd_ic").id)
    close_loops(m, d)
    q0 = d.qpos.copy()
    run(2.0)
    check("C1 hold at rmd IC, 2 s (gravcomp)       [rad]", np.abs(d.qpos - q0).max(), 1e-8)

    rng = np.random.default_rng(0)
    err, drift = 0.0, 0.0
    for _ in range(3):
        lo, hi = m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1]
        d.ctrl[:] = np.clip(d.ctrl + rng.uniform(-0.6, 0.6, m.nu), lo, hi)
        drift = max(drift, run(d.time + 1.5))
        err = max(err, np.abs(d.qpos[qa] - d.ctrl).max())
    check("C2 step +-0.6 rad, error after 1.5 s    [rad]", err, 1e-4)
    check("C2 loop drift during steps              [m]", drift, 1e-4)

    # q6 to each end of its slider: the parallelogram must stay on its branch
    # (q6_1 = -q6, q6_2 = +q6 for an ideal parallelogram; not crossed)
    worst = 0.0
    for end in (0, 1):
        mj.mj_resetDataKeyframe(m, d, m.key("rmd_ic").id)
        close_loops(m, d)
        for s in ARMS:
            i = m.actuator(f"{s}_servo6").id
            d.ctrl[i] = m.actuator_ctrlrange[i, end]
        run(3.0)
        for s in ARMS:
            q6 = d.qpos[m.jnt_qposadr[m.joint(f"{s}_joint6").id]]
            q61 = d.qpos[m.jnt_qposadr[m.joint(f"{s}_joint6_1").id]]
            worst = max(worst, abs(q61 + q6))
    check("C3 q6 at slider ends: linkage on branch  [rad]", worst, 0.05)


def main() -> int:
    part_a()
    part_b()
    part_c()
    bad = [r for r in results if not r[3]]
    print(f"\n{len(results) - len(bad)}/{len(results)} checks passed")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
