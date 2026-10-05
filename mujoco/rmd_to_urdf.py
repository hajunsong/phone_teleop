"""HumanoidUpperBody.rmd  ->  URDF + STL meshes + loop/actuator sidecar.

Single source of truth is the .rmd.  Nothing kinematic or inertial is typed in
here; every number below is read from it (via ../chrono/rmd_model.py, the
Python port of matlab/parse_rmd.m).

Frames
------
* The rmd assembly pose is q = 0: every joint's I and J markers coincide there
  (position and full orientation), which this script re-checks.
* Link frame of a serial-chain body = its 'Ai' marker.  Ai shares origin and z
  axis with the joint's I marker, so the joint axis is +z and the MuJoCo body
  pose equals the MATLAB/C++ A_i (params/arm_*.json) one-for-one.
* Bodies without an Ai (the wrist parallel linkage body6_1/body6_2) use their
  inbound joint's I marker.  'base' uses its part frame (= model global).
* RecurDyn joint direction: I marker on the child, J marker on the parent.  The
  tree is grown along that direction from 'base'; a joint whose I-body already
  has a parent closes a loop.  URDF cannot hold loops, so those go to the
  sidecar JSON and urdf_to_mjcf.py turns them into <equality connect>.
* Ground is folded into 'base': base is welded to Ground with identity offset
  (checked), so RA body0 (welded to Ground) and LA body0 (welded to base) both
  hang off 'base' in the URDF.

Units: rmd is MMKS; URDF/MJCF are SI.  Conversion happens only here.

Usage:  python rmd_to_urdf.py [--rmd PATH] [--out DIR]
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import struct
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
CLAUDE_WS = HERE.parent
ROOT = CLAUDE_WS.parent
sys.path.insert(0, str(CLAUDE_WS / "chrono"))
import rmd_model  # noqa: E402

SIDES = {"@RightArm": "RA", "@LeftArm": "LA"}
MESH_DIRS = [CLAUDE_WS / "motion_generation" / "viewer" / "cad" / "mesh",
             CLAUDE_WS / "InverseKinematics" / "cad" / "mesh"]
SUBSYSTEM_OF_SIDE = {"RA": "RightArm", "LA": "LeftArm", "": "HumanoidUpperBody"}


# --------------------------------------------------------------------------
def split_name(name: str) -> tuple[str, str]:
    """'body3@RightArm' -> ('RA', 'body3');  'base' -> ('', 'base')."""
    for suf, side in SIDES.items():
        if name.endswith(suf):
            return side, name[: -len(suf)]
    return "", name


def link_name(part_name: str) -> str:
    side, short = split_name(part_name)
    return f"{side}_{short}" if side else short


def joint_label(joint_name: str) -> str:
    """'RevJoint6_1@RightArm' -> 'RA_joint6_1';  'RevJoint8@LeftArm' kept as is."""
    side, short = split_name(joint_name)
    short = short.replace("RevJoint", "joint").replace("Fixed", "fixed")
    return f"{side}_{short}" if side else short


def T_of(p, R) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, p
    return T


def inv(T) -> np.ndarray:
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def rpy(R) -> tuple[float, float, float]:
    """URDF fixed-axis roll-pitch-yaw:  R = Rz(y) Ry(p) Rx(r)."""
    sp = -R[2, 0]
    if abs(sp) < 1 - 1e-12:
        p = math.asin(sp)
        r = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(R[1, 0], R[0, 0])
    else:  # gimbal lock: put everything in yaw
        p = math.copysign(math.pi / 2, sp)
        r = 0.0
        y = math.atan2(-R[0, 1], R[1, 1])
    return r, p, y


def rpy_to_R(r, p, y) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = map(lambda f: f, (math.cos(r), math.sin(r), math.cos(p),
                                              math.sin(p), math.cos(y), math.sin(y)))
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def fmt(v) -> str:
    return " ".join(f"{x:.17g}" for x in np.asarray(v, float).ravel())


def make_physical(I: np.ndarray) -> tuple[np.ndarray, dict | None]:
    """Minimal repair of a rigid-body inertia that breaks A + B >= C.

    Any real mass distribution satisfies the triangle inequality on its
    principal moments; the rmd's body6_1 does not (a CAD mass-property artefact).
    RecurDyn integrates it anyway, MuJoCo refuses it.  Only the smallest
    principal moment is raised, to C - B, in the principal frame, so
    the principal axes and the two larger moments are untouched.
    """
    e, V = np.linalg.eigh(I)                              # ascending
    viol = e[2] - e[1] - e[0]
    if viol <= 0:
        return I, None
    e2 = e.copy()
    e2[0] = e[2] - e[1] + 1e-5 * e[2]    # margin: MJCF writes ~8 digits
    I2 = V @ np.diag(e2) @ V.T
    return 0.5 * (I2 + I2.T), dict(eig_before=e.tolist(), eig_after=e2.tolist(),
                                    violation_rel=float(viol / e[2]))


# --------------------------------------------------------------------------
def write_stl(path: Path, V: np.ndarray, F: np.ndarray) -> None:
    """Binary STL.  V [m] in link frame, F int triangles."""
    tri = V[F]                                            # (n,3,3)
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)
    rec = np.zeros(len(F), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tri
    with open(path, "wb") as f:
        f.write(b"rmd_to_urdf".ljust(80, b" "))
        f.write(struct.pack("<I", len(F)))
        f.write(rec.tobytes())


def load_mesh_manifest() -> tuple[Path, dict]:
    for d in MESH_DIRS:
        man = d / "mesh_manifest.csv"
        if man.exists():
            with open(man, newline="", encoding="utf-8") as f:
                rows = {(r["subsystem"], r["body"]): d / r["npz"] for r in csv.DictReader(f)}
            return d, rows
    return MESH_DIRS[0], {}


# --------------------------------------------------------------------------
def build(rmd_path: Path, out: Path) -> dict:
    m = rmd_model.load(str(rmd_path))
    L = m.length_to_m()
    ground = next(p.id for p in m.parts.values() if p.is_ground)
    base = next(p.id for p in m.parts.values() if p.name == "base")

    def mk_world(mid):
        p, R = m.marker_world(mid)
        return T_of(p, R)

    # ---- sanity: assembly pose is q = 0 ---------------------------------
    for j in m.joints.values():
        TI, TJ = mk_world(j.I), mk_world(j.J)
        if np.abs(TI - TJ).max() > 1e-6:
            raise RuntimeError(f"{j.name}: I/J markers do not coincide at assembly; "
                               "the q=0 convention used here would be wrong.")

    # ---- markers per part, by short name --------------------------------
    by_part: dict[int, dict[str, int]] = {}
    for mk in m.markers.values():
        short = mk.name.split("@")[0].split(".", 1)[-1]
        by_part.setdefault(mk.part, {})[short] = mk.id

    def part_of(mid):
        pid = m.markers[mid].part
        return base if pid == ground else pid          # fold Ground into base

    # ---- spanning tree along J(parent) -> I(child) -----------------------
    children: dict[int, list] = {}
    for j in sorted(m.joints.values(), key=lambda j: j.id):
        children.setdefault(part_of(j.J), []).append(j)

    parent_joint: dict[int, object] = {}
    loops, order, stack = [], [base], [base]
    while stack:
        pid = stack.pop(0)
        for j in children.get(pid, []):
            c = part_of(j.I)
            if c == base and pid == base:              # base <-> Ground weld
                if np.abs(mk_world(j.I) - mk_world(j.J)).max() > 1e-9 or \
                   np.abs(T_of(m.parts[base].p, m.parts[base].R) - np.eye(4)).max() > 1e-9:
                    raise RuntimeError("base is not welded to Ground at identity")
                continue
            if c in parent_joint or c == base:
                loops.append(j)
                continue
            parent_joint[c] = j
            order.append(c)
            stack.append(c)

    orphans = [p.name for p in m.parts.values()
               if not p.is_ground and p.id != base and p.id not in parent_joint]
    if orphans:
        raise RuntimeError(f"parts not reachable from base: {orphans}")

    # ---- link frames (world, at assembly) --------------------------------
    T_link: dict[int, np.ndarray] = {base: T_of(m.parts[base].p, m.parts[base].R)}
    frame_src: dict[int, str] = {base: "part frame"}
    for pid in order[1:]:
        mks = by_part[pid]
        if "Ai" in mks:
            T_link[pid], frame_src[pid] = mk_world(mks["Ai"]), m.markers[mks["Ai"]].name
        else:
            T_link[pid], frame_src[pid] = mk_world(parent_joint[pid].I), \
                m.markers[parent_joint[pid].I].name
        j = parent_joint[pid]
        if j.kind == "Revolute":
            zI = mk_world(j.I)[:3, 2]
            TL = T_link[pid]
            if np.linalg.norm(TL[:3, 3] - mk_world(j.I)[:3, 3]) > 1e-9 or \
               abs(TL[:3, 2] @ zI - 1) > 1e-12:
                raise RuntimeError(f"{m.parts[pid].name}: link frame not on joint axis")

    # ---- meshes -----------------------------------------------------------
    mesh_dir_src, manifest = load_mesh_manifest()
    (out / "meshes").mkdir(parents=True, exist_ok=True)
    mesh_file: dict[int, str] = {}
    mesh_stats = []
    for pid in order:
        side, short = split_name(m.parts[pid].name)
        src = manifest.get((SUBSYSTEM_OF_SIDE[side], short))
        if src is None or not src.exists():
            continue
        d = np.load(src)
        V = d["V"].astype(np.float64) * L                          # part frame, m
        Tp = T_of(m.parts[pid].p * L, m.parts[pid].R)
        Tl = T_link[pid].copy(); Tl[:3, 3] *= L
        M = inv(Tl) @ Tp
        Vl = V @ M[:3, :3].T + M[:3, 3]
        name = f"{link_name(m.parts[pid].name)}.stl"
        write_stl(out / "meshes" / name, Vl, d["F"].astype(np.int64))
        mesh_file[pid] = name
        mesh_stats.append((name, len(d["F"])))

    # ---- URDF -------------------------------------------------------------
    sha = hashlib.sha256(rmd_path.read_bytes()).hexdigest()
    X = ['<?xml version="1.0"?>',
         f"<!-- Generated by claude_ws/mujoco/rmd_to_urdf.py from {rmd_path.name}",
         f"     sha256 {sha}",
         "     Do not edit by hand: rerun the script.  Units SI.",
         "     Wrist parallel-linkage loop closures are NOT in this file (URDF is a tree);",
         "     see humanoid_upper.loops.json / the MJCF. -->",
         '<robot name="HumanoidUpperBody">',
         "  <mujoco>",
         '    <compiler angle="radian" balanceinertia="false" discardvisual="true"'
         ' fusestatic="false" strippath="false"/>',
         "  </mujoco>"]

    inertial_rep = {}
    for pid in order:
        pt = m.parts[pid]
        name = link_name(pt.name)
        X.append(f'  <link name="{name}">')
        X.append(f"    <!-- RecurDyn part '{pt.name}', frame = {frame_src[pid]} -->")
        if pt.mass > 0:
            Tcm = mk_world(pt.cm_marker)
            Tl = T_link[pid]
            c = Tl[:3, :3].T @ (Tcm[:3, 3] - Tl[:3, 3]) * L
            Rc = Tl[:3, :3].T @ Tcm[:3, :3]
            I = Rc @ pt.inertia_cm @ Rc.T * L * L
            I = 0.5 * (I + I.T)
            I_rmd = I
            I, fix = make_physical(I)
            inertial_rep[name] = dict(mass=pt.mass, com=c.tolist(), inertia=I.tolist())
            if fix:
                inertial_rep[name]["repaired"] = dict(fix, inertia_rmd=I_rmd.tolist())
                print(f"WARNING {name} ('{pt.name}'): rmd inertia violates the triangle "
                      f"inequality by {fix['violation_rel']:.1%}; smallest principal moment "
                      f"raised {fix['eig_before'][0]:.4g} -> {fix['eig_after'][0]:.4g} kg*m^2")
            X += ["    <inertial>",
                  f'      <origin xyz="{fmt(c)}" rpy="0 0 0"/>',
                  f'      <mass value="{pt.mass:.17g}"/>',
                  f'      <inertia ixx="{I[0,0]:.17g}" ixy="{I[0,1]:.17g}" ixz="{I[0,2]:.17g}"'
                  f' iyy="{I[1,1]:.17g}" iyz="{I[1,2]:.17g}" izz="{I[2,2]:.17g}"/>',
                  "    </inertial>"]
        if pid in mesh_file:
            for tag in ("visual", "collision"):
                X += [f"    <{tag}>",
                      '      <origin xyz="0 0 0" rpy="0 0 0"/>',
                      f'      <geometry><mesh filename="../meshes/{mesh_file[pid]}"/></geometry>',
                      f"    </{tag}>"]
        X.append("  </link>")

    for pid in order[1:]:
        j = parent_joint[pid]
        par = part_of(j.J)
        Tpc = inv(T_link[par]) @ T_link[pid]
        xyz = Tpc[:3, 3] * L
        r, p, y = rpy(Tpc[:3, :3])
        if np.abs(rpy_to_R(r, p, y) - Tpc[:3, :3]).max() > 1e-12:
            raise RuntimeError(f"rpy round trip failed for {j.name}")
        kind = {"Revolute": "continuous", "Fixed": "fixed"}.get(j.kind)
        if kind is None:
            raise RuntimeError(f"{j.name}: joint type {j.kind} not handled")
        X.append(f'  <joint name="{joint_label(j.name)}" type="{kind}">')
        X.append(f"    <!-- RecurDyn '{j.name}'  I={m.markers[j.I].name}"
                 f"  J={m.markers[j.J].name} -->")
        X += [f'    <parent link="{link_name(m.parts[par].name)}"/>',
              f'    <child link="{link_name(m.parts[pid].name)}"/>',
              f'    <origin xyz="{fmt(xyz)}" rpy="{r:.17g} {p:.17g} {y:.17g}"/>']
        if kind == "continuous":
            X.append('    <axis xyz="0 0 1"/>')
        X.append("  </joint>")

    # tool frames: body7.Cij (the frame params/arm_*.json calls 'tool')
    tools = {}
    for pid in order:
        side, short = split_name(m.parts[pid].name)
        if side and short == "body7" and "Cij" in by_part[pid]:
            Tt = inv(T_link[pid]) @ mk_world(by_part[pid]["Cij"])
            r, p, y = rpy(Tt[:3, :3])
            tname = f"{side}_tool"
            tools[side] = dict(link=tname, parent=link_name(m.parts[pid].name),
                               pos=(Tt[:3, 3] * L).tolist(), R=Tt[:3, :3].tolist())
            X += [f'  <link name="{tname}"/>',
                  f'  <joint name="{side}_tool_fixed" type="fixed">',
                  f"    <!-- RecurDyn marker '{m.markers[by_part[pid]['Cij']].name}' -->",
                  f'    <parent link="{link_name(m.parts[pid].name)}"/>',
                  f'    <child link="{tname}"/>',
                  f'    <origin xyz="{fmt(Tt[:3, 3] * L)}" rpy="{r:.17g} {p:.17g} {y:.17g}"/>',
                  "  </joint>"]
    X.append("</robot>")

    (out / "urdf").mkdir(parents=True, exist_ok=True)
    urdf_path = out / "urdf" / "humanoid_upper.urdf"
    urdf_path.write_text("\n".join(X) + "\n", encoding="utf-8")

    # ---- sidecar: what URDF cannot carry --------------------------------
    loop_rep = []
    for j in loops:
        b1, b2 = part_of(j.I), part_of(j.J)
        Tw = mk_world(j.I)
        a1 = (inv(T_link[b1]) @ Tw)[:3, 3] * L
        a2 = (inv(T_link[b2]) @ Tw)[:3, 3] * L
        loop_rep.append(dict(name=joint_label(j.name), rmd_name=j.name, kind=j.kind,
                             body1=link_name(m.parts[b1].name),
                             body2=link_name(m.parts[b2].name),
                             anchor1=a1.tolist(), anchor2=a2.tolist(),
                             axis_world=Tw[:3, 2].tolist()))

    act_rep = []
    for a in sorted(m.axial.values(), key=lambda a: a.id):
        cI, cJ = part_of(a.I), part_of(a.J)
        j = parent_joint.get(cI)
        if j is None or part_of(j.J) != cJ:
            raise RuntimeError(f"{a.name}: I/J bodies do not match a tree joint")
        zA = mk_world(a.I)[:3, 2]
        zJ = mk_world(j.I)[:3, 2]
        if not a.rotation or abs(zA @ zJ - 1) > 1e-9:
            raise RuntimeError(f"{a.name}: not a rotational axial force about +z of {j.name}")
        act_rep.append(dict(name=joint_label(j.name).replace("joint", "motor"),
                            rmd_name=a.name, joint=joint_label(j.name), function=a.function))

    ic = {joint_label(j.name): j.ic[1] for j in m.joints.values()
          if j.ic and j.kind == "Revolute" and j not in loops}

    sidecar = dict(
        source=str(rmd_path), sha256=sha, units="SI (m, kg, s, rad, N, N*m)",
        gravity=(m.gravity * L).tolist(),
        loops=loop_rep, actuators=act_rep, initial_q=ic, tools=tools,
        frames={link_name(m.parts[p].name): frame_src[p] for p in order},
        inertial=inertial_rep, meshes=dict(source=str(mesh_dir_src), files=mesh_stats))
    (out / "urdf" / "humanoid_upper.loops.json").write_text(
        json.dumps(sidecar, indent=2, ensure_ascii=False), encoding="utf-8")
    return sidecar


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rmd", type=Path, default=ROOT / "HumanoidUpperBody.rmd")
    ap.add_argument("--out", type=Path, default=HERE)
    a = ap.parse_args()
    s = build(a.rmd.resolve(), a.out.resolve())
    print(f"URDF   {a.out / 'urdf' / 'humanoid_upper.urdf'}")
    print(f"links  {len(s['frames'])}   meshes {len(s['meshes']['files'])}"
          f"   loops {[l['name'] for l in s['loops']]}")
    print(f"motors {len(s['actuators'])}   IC {s['initial_q']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
