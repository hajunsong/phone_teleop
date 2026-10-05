"""Closed-loop helpers for the wrist parallel linkage (shared by verify/view/urdf_to_mjcf).

The MJCF closes each arm's parallelogram with one <equality connect>.  These
work on any model built by urdf_to_mjcf.py (torque or servo variant).
"""

from __future__ import annotations

import mujoco as mj
import numpy as np

ARMS = ("RA", "LA")


def loop_residual(m, d):
    r = []
    for k in range(m.neq):
        b1, b2 = m.eq_obj1id[k], m.eq_obj2id[k]
        p1 = d.xpos[b1] + d.xmat[b1].reshape(3, 3) @ m.eq_data[k, 0:3]
        p2 = d.xpos[b2] + d.xmat[b2].reshape(3, 3) @ m.eq_data[k, 3:6]
        r.append(p1 - p2)
    return np.concatenate(r)


def passive_adr(m):
    names = [f"{s}_joint6_{i}" for s in ARMS for i in (1, 2)]
    return np.array([m.jnt_qposadr[m.joint(n).id] for n in names]), \
           np.array([m.jnt_dofadr[m.joint(n).id] for n in names])


def close_loops(m, d, iters=30):
    """Solve the passive linkage angles for the current actuated qpos."""
    pa, _ = passive_adr(m)
    for s in ARMS:                                   # parallelogram guess
        q6 = d.qpos[m.jnt_qposadr[m.joint(f"{s}_joint6").id]]
        d.qpos[m.jnt_qposadr[m.joint(f"{s}_joint6_1").id]] = -q6
        d.qpos[m.jnt_qposadr[m.joint(f"{s}_joint6_2").id]] = q6
    for _ in range(iters):
        mj.mj_kinematics(m, d)
        r = loop_residual(m, d)
        if np.abs(r).max() < 1e-14:
            break
        J = np.zeros((len(r), len(pa)))
        for i, a in enumerate(pa):
            d.qpos[a] += 1e-7
            mj.mj_kinematics(m, d)
            J[:, i] = (loop_residual(m, d) - r) / 1e-7
            d.qpos[a] -= 1e-7
        d.qpos[pa] -= np.linalg.lstsq(J, r, rcond=None)[0]
    mj.mj_kinematics(m, d)
    return np.abs(loop_residual(m, d)).max()


def loop_gravity_torque(m, d):
    """Actuated torques holding the closed-loop model still (qd = qdd = 0).

    qfrc_bias = tau + J' lam, tau zero on the passive joints.  The passive rows
    fix lam up to the out-of-plane component, which produces no generalized
    force (the loop is planar), so tau is unique.
    """
    d.qvel[:] = 0
    mj.mj_forward(m, d)
    G = d.qfrc_bias.copy()
    J = []
    for k in range(m.neq):
        b1, b2 = m.eq_obj1id[k], m.eq_obj2id[k]
        pt = d.xpos[b1] + d.xmat[b1].reshape(3, 3) @ m.eq_data[k, 0:3]
        j1, j2 = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        mj.mj_jac(m, d, j1, None, pt, b1)
        mj.mj_jac(m, d, j2, None, pt, b2)
        J.append(j1 - j2)
    J = np.vstack(J)
    _, pd = passive_adr(m)
    lam = np.linalg.lstsq(J[:, pd].T, G[pd], rcond=None)[0]
    tau = G - J.T @ lam
    return tau, np.abs(tau[pd]).max()


def loop_sigma(m, d, k):
    """Smallest singular value of loop k's constraint Jacobian w.r.t. its passive
    joints [m/rad].  Zero = the parallelogram is at a singular (folded) pose."""
    b1, b2 = m.eq_obj1id[k], m.eq_obj2id[k]
    pt = d.xpos[b1] + d.xmat[b1].reshape(3, 3) @ m.eq_data[k, 0:3]
    j1, j2 = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    mj.mj_jac(m, d, j1, None, pt, b1)
    mj.mj_jac(m, d, j2, None, pt, b2)
    side = m.body(b1).name.split("_")[0]
    pd = [m.jnt_dofadr[m.joint(f"{side}_joint6_{i}").id] for i in (1, 2)]
    return np.linalg.svd((j1 - j2)[:, pd], compute_uv=False)[-1]


def q6_singular_limits(m, step_deg=0.25, span_deg=180.0):
    """Nearest singular q6 on each side of 0, per arm: {side: (neg, pos)} [rad].

    Scans outward from the assembly pose, solving the loop at each step, and
    stops at the first local minimum of loop_sigma that is below 5 % of its value
    at q6 = 0.
    """
    d = mj.MjData(m)
    out = {}
    for k in range(m.neq):
        side = m.body(m.eq_obj1id[k]).name.split("_")[0]
        qa = m.jnt_qposadr[m.joint(f"{side}_joint6").id]

        def sigma(q):
            mj.mj_resetData(m, d)
            d.qpos[qa] = q
            close_loops(m, d)
            mj.mj_kinematics(m, d)
            mj.mj_comPos(m, d)
            return loop_sigma(m, d, k)

        s0 = sigma(0.0)
        lim = []
        for sgn in (-1, 1):
            qs = np.radians(np.arange(0, span_deg + step_deg, step_deg)) * sgn
            s = np.array([sigma(q) for q in qs])
            idx = next((i for i in range(1, len(s) - 1)
                        if s[i] <= s[i - 1] and s[i] <= s[i + 1] and s[i] < 0.05 * s0), None)
            lim.append(qs[idx] if idx is not None else qs[-1])
        out[side] = tuple(lim)
    return out
