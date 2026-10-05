"""URDF + sidecar  ->  MuJoCo MJCF (full precision), split like the RecurDyn subsystems.

The URDF (rmd_to_urdf.py) is a tree.  This adds what URDF cannot express,
all of it read from urdf/humanoid_upper.loops.json (which came from the rmd):

  * loop closures        RevJoint6_3@RightArm / RevJoint8@LeftArm  -> <equality connect>
  * actuators            RotationalAxial1..7 per arm               -> <motor>, gear 1, N*m
  * initial pose         joint IC displacements                    -> keyframe 'rmd_ic'
  * gravity              ACCGRAV                                    -> option/gravity
  * tool frames          body7.Cij (massless URDF leaf links)       -> site + frame sensors

File layout mirrors the rmd's subsystems (HumanoidUpperBody / RightArm / LeftArm):

  mjcf/arm_RA.xml           RightArm subsystem, in its own frame (body0 at the
  mjcf/arm_LA.xml           LeftArm   origin), names without the side prefix;
                            loadable on its own
  mjcf/humanoid_upper.xml   base + <frame RA_mount/LA_mount> at the subsystem
                            frame (T_B0) + <attach model=.. prefix="RA_"/"LA_">,
                            keyframes
  mjcf/*_servo.xml          same layout, slider variant (see servo_files)
  mjcf/scene*.xml           + floor/light

The two arms are separate files, not one file attached twice: they are mirror
images and their rmd data differ slightly (body3 CoM, linkage dimensions).

Checks run on every build:
  * the split model, once attached, must equal the single-file model written
    from the same URDF, name by name (bodies, joints, geoms, sites, equality,
    actuators, sensors, keyframes, options, and the kinematics at rmd_ic);
  * that model must agree with MuJoCo's own URDF importer (check_against_native).

Why not MjSpec.to_xml(): it prints 6 significant digits, which puts ~1e-6
errors into every frame.  Everything here is written with 17 digits.

Contacts are off: the rmd defines none, and the CAD meshes of neighbouring
links interpenetrate at the joints.  Joint damping/friction/armature are 0 for
the same reason (the RecurDyn revolute joints carry none).

The parallel linkage is planar (all four axes parallel), so a point (connect)
constraint is enough; its out-of-plane row is redundant and MuJoCo's soft
constraints absorb that.

Usage:  python urdf_to_mjcf.py [--dt 5e-4] [--loop-solref 0.002 1] [--servo-hz 4]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco as mj
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import loops  # noqa: E402

HERE = Path(__file__).resolve().parent
URDF = HERE / "urdf" / "humanoid_upper.urdf"
SIDECAR = HERE / "urdf" / "humanoid_upper.loops.json"
OUT = HERE / "mjcf"
SUBSYSTEM = {"RA": "RightArm", "LA": "LeftArm"}

COLORS = {"base": "0.62 0.64 0.68 1", "RA": "0.80 0.45 0.30 1",
          "LA": "0.30 0.52 0.80 1", "link": "0.85 0.85 0.88 1"}

# Scene extras (floor, light, camera).  Merged into a copy of the top file rather
# than <include>-ing it: MuJoCo 3.13 resolves an included file's <model file=..>
# against the directory twice when the scene is opened by a relative path
# ('mjcf/scene.xml' -> 'mjcf/mjcf/arm_RA.xml').
SCENE_HEAD = """  <statistic center="0 0 0.1" extent="1.2"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <global azimuth="150" elevation="-20" offwidth="1280" offheight="960"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4"
             rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>
"""
SCENE_WORLD = """    <light pos="0 0 2.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" pos="0 0 -0.75" type="plane" material="groundplane"
          contype="0" conaffinity="0"/>
"""


def make_scene(top: str) -> str:
    """Top file + floor/light/camera, self-contained (see SCENE_HEAD)."""
    out = top.replace("  <asset>\n", SCENE_HEAD + "  <asset>\n", 1)
    out = out.replace("  <worldbody>\n", "  <worldbody>\n" + SCENE_WORLD, 1)
    return out.replace('<mujoco model="HumanoidUpperBody', '<mujoco model="scene: HumanoidUpperBody', 1)


def f17(v) -> str:
    return " ".join(f"{float(x):.17g}" for x in np.ravel(v))


def rpy_to_R(r, p, y) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def R_to_quat(R) -> np.ndarray:
    q = np.zeros(4)
    mj.mju_mat2Quat(q, np.ascontiguousarray(R).ravel())
    return q / np.linalg.norm(q)


def origin(el) -> tuple[np.ndarray, np.ndarray]:
    o = el.find("origin") if el is not None else None
    if o is None:
        return np.zeros(3), np.eye(3)
    xyz = np.array([float(x) for x in o.get("xyz", "0 0 0").split()])
    r, p, y = (float(x) for x in o.get("rpy", "0 0 0").split())
    return xyz, rpy_to_R(r, p, y)


# --------------------------------------------------------------------------
def parse_urdf(path: Path):
    root = ET.parse(path).getroot()
    links = {l.get("name"): l for l in root.findall("link")}
    joints = root.findall("joint")
    child_of = {j.find("child").get("link"): j for j in joints}
    kids: dict[str, list] = {}
    for j in joints:
        kids.setdefault(j.find("parent").get("link"), []).append(j)
    roots = [n for n in links if n not in child_of]
    if len(roots) != 1:
        raise RuntimeError(f"URDF must have one root link, found {roots}")
    return links, kids, roots[0]


def is_massless_leaf(link, kids) -> bool:
    return (link.find("inertial") is None and link.find("collision") is None
            and link.find("visual") is None and not kids.get(link.get("name")))


def material_of(local: str, full: str) -> str:
    """base -> 'base'; linkage bodies -> 'link'; other arm bodies -> 'arm' in a
    subsystem file, or the side ('RA'/'LA') in the single-file model."""
    if full == "base":
        return "base"
    if full.endswith(("6_1", "6_2")):
        return "link"
    return "arm" if local != full else full.split("_")[0]


def emit_tree(links, kids, name, ind, lines, meshes, rename=lambda n: n, joint_el=None,
              mount=None, root_at_origin=False):
    """Write link `name` and its subtree as MJCF <body> lines.

    rename          URDF name -> name used in this file (subsystem files drop the
                    'RA_'/'LA_' prefix; <attach prefix=...> puts it back)
    mount           {child link: callback(joint_el, ind)}: instead of descending into
                    that child, the callback writes what goes there (<frame><attach>)
    root_at_origin  the subsystem root sits at its file's origin (its own frame)
    """
    link = links[name]
    if joint_el is None or root_at_origin:
        head = f'<body name="{rename(name)}">'
    else:
        p, R = origin(joint_el)
        head = f'<body name="{rename(name)}" pos="{f17(p)}" quat="{f17(R_to_quat(R))}">'
    lines.append(ind + head)
    inn = ind + "  "

    ine = link.find("inertial")
    if ine is not None:
        c, Rc = origin(ine)
        i = ine.find("inertia")
        I = np.array([[float(i.get("ixx")), float(i.get("ixy")), float(i.get("ixz"))],
                      [float(i.get("ixy")), float(i.get("iyy")), float(i.get("iyz"))],
                      [float(i.get("ixz")), float(i.get("iyz")), float(i.get("izz"))]])
        I = Rc @ I @ Rc.T                               # into the link frame
        mass = float(ine.find("mass").get("value"))
        # Diagonalise here in float64: MuJoCo's own fullinertia path (mju_eig3)
        # is iterative and leaves ~1e-8 relative error in the tensor.
        w, V = np.linalg.eigh(0.5 * (I + I.T))
        if np.linalg.det(V) < 0:
            V[:, 0] = -V[:, 0]
        lines.append(inn + f'<inertial pos="{f17(c)}" quat="{f17(R_to_quat(V))}" '
                     f'mass="{mass:.17g}" diaginertia="{f17(w)}"/>')

    if joint_el is not None and joint_el.get("type") in ("revolute", "continuous"):
        ax = joint_el.find("axis")
        axis = ax.get("xyz") if ax is not None else "1 0 0"
        lines.append(inn + f'<joint name="{rename(joint_el.get("name"))}" type="hinge" axis="{axis}"/>')
    elif joint_el is not None and joint_el.get("type") != "fixed":
        raise RuntimeError(f"joint type {joint_el.get('type')} not handled")

    col = link.find("collision")
    if col is not None:
        fn = col.find("geometry/mesh").get("filename")
        p, R = origin(col)
        mname = rename(Path(fn).stem)
        meshes.append((mname, Path(fn).name))
        lines.append(inn + f'<geom name="{rename(name)}" type="mesh" mesh="{mname}" pos="{f17(p)}" '
                     f'quat="{f17(R_to_quat(R))}" material="{material_of(rename(name), name)}"/>')

    for j in kids.get(name, []):
        cname = j.find("child").get("link")
        if mount and cname in mount:
            mount[cname](j, inn)
        elif is_massless_leaf(links[cname], kids) and j.get("type") == "fixed":
            p, R = origin(j)
            lines.append(inn + f'<site name="{rename(cname)}" pos="{f17(p)}" '
                         f'quat="{f17(R_to_quat(R))}" size="0.006" rgba="1 0.9 0.1 1" group="2"/>')
        else:
            emit_tree(links, kids, cname, inn, lines, meshes, rename, j)
    lines.append(ind + "</body>")


def header(side, what):
    return ["<!-- Generated by claude_ws/mujoco/urdf_to_mjcf.py from urdf/humanoid_upper.urdf",
            f"     {what}",
            f"     rmd sha256 {side['sha256']}",
            "     Do not edit by hand: rerun rmd_to_urdf.py then urdf_to_mjcf.py. -->"]


def common_head(side, dt, model_name):
    """compiler/option/default, identical in every file so <attach> has no conflicts
    and a subsystem file opened on its own behaves the same."""
    return [f'<mujoco model="{model_name}">',
            '  <compiler angle="radian" meshdir="../meshes" autolimits="true"/>',
            f'  <option timestep="{dt:.17g}" gravity="{f17(side["gravity"])}" integrator="implicitfast">',
            '    <flag contact="disable"/>',
            "  </option>",
            "  <default>",
            '    <geom contype="0" conaffinity="0" group="1"/>',
            '    <joint damping="0" armature="0" frictionloss="0"/>',
            "  </default>"]


def arm_sections(side, arm, rename, loop_solref, loop_solimp):
    """<equality>/<actuator>/<sensor> for one arm (arm=None: both, single file)."""
    mine = (lambda n: n.startswith(f"{arm}_")) if arm else (lambda n: True)
    X = ["  <equality>"]
    for lp in side["loops"]:
        if not mine(lp["name"]):
            continue
        X.append(f'    <!-- RecurDyn {lp["rmd_name"]}: revolute about x, closed as a point '
                 "(planar loop) -->")
        X.append(f'    <connect name="{rename(lp["name"])}" body1="{rename(lp["body1"])}" '
                 f'body2="{rename(lp["body2"])}" anchor="{f17(lp["anchor1"])}" '
                 f'solref="{f17(loop_solref)}" solimp="{f17(loop_solimp)}"/>')
    X.append("  </equality>")

    acts = [a for a in side["actuators"] if mine(a["joint"])]
    X.append("  <actuator>")
    for a in acts:
        X.append(f'    <motor name="{rename(a["name"])}" joint="{rename(a["joint"])}" gear="1"/>'
                 f'  <!-- {a["rmd_name"]} FUNCTION={a["function"]} -->')
    X.append("  </actuator>")

    X.append("  <sensor>")
    for s in sorted(side["tools"]):
        if arm and s != arm:
            continue
        X.append(f'    <framepos name="{rename(s + "_tool_pos")}" objtype="site" '
                 f'objname="{rename(s + "_tool")}"/>')
        X.append(f'    <framequat name="{rename(s + "_tool_quat")}" objtype="site" '
                 f'objname="{rename(s + "_tool")}"/>')
    for a in acts:
        X.append(f'    <jointpos name="{rename(a["joint"] + "_q")}" joint="{rename(a["joint"])}"/>')
        X.append(f'    <jointvel name="{rename(a["joint"] + "_qd")}" joint="{rename(a["joint"])}"/>')
        X.append(f'    <actuatorfrc name="{rename(a["name"] + "_tau")}" actuator="{rename(a["name"])}"/>')
    X.append("  </sensor>")
    return X


# --------------------------------------------------------------------------
def write_single(dt, loop_solref, loop_solimp) -> str:
    """Single-file model: the reference the subsystem split must reproduce."""
    side = json.loads(SIDECAR.read_text(encoding="utf-8"))
    links, kids, root = parse_urdf(URDF)
    meshes, lines = [], []
    emit_tree(links, kids, root, "    ", lines, meshes)
    X = header(side, "single-file reference (compiled in memory only)") + \
        common_head(side, dt, "HumanoidUpperBody") + ["  <asset>"]
    X += [f'    <material name="{k}" rgba="{v}"/>' for k, v in COLORS.items()]
    X += [f'    <mesh name="{n}" file="{fn}"/>' for n, fn in meshes]
    X += ["  </asset>", "  <worldbody>"] + lines + ["  </worldbody>"]
    X += arm_sections(side, None, lambda n: n, loop_solref, loop_solimp)
    X.append("</mujoco>")
    return "\n".join(X) + "\n"


def write_split(dt, loop_solref, loop_solimp) -> dict[str, str]:
    """{filename: text} for arm_RA.xml, arm_LA.xml and humanoid_upper.xml (no keyframes yet)."""
    side = json.loads(SIDECAR.read_text(encoding="utf-8"))
    links, kids, root = parse_urdf(URDF)
    arms = sorted(side["tools"], reverse=True)          # RA, LA: the URDF/rmd order
    files: dict[str, str] = {}
    top_lines, top_meshes = [], []

    def attach(arm):
        def write(joint_el, ind):
            p, R = origin(joint_el)
            top_lines.append(ind + f'<frame name="{arm}_mount" pos="{f17(p)}" '
                             f'quat="{f17(R_to_quat(R))}">  <!-- {SUBSYSTEM[arm]} subsystem frame -->')
            top_lines.append(ind + f'  <attach model="{arm}" body="body0" prefix="{arm}_"/>')
            top_lines.append(ind + "</frame>")
        return write

    emit_tree(links, kids, root, "    ", top_lines, top_meshes,
              mount={f"{a}_body0": attach(a) for a in arms})

    for arm in arms:
        pre = f"{arm}_"
        rename = lambda n, pre=pre: n[len(pre):] if n.startswith(pre) else n
        lines, meshes = [], []
        emit_tree(links, kids, f"{arm}_body0", "    ", lines, meshes, rename, root_at_origin=True)
        X = header(side, f"{SUBSYSTEM[arm]} subsystem. Attached by humanoid_upper.xml with "
                         f"prefix '{arm}_'; opens on its own in its subsystem frame.") + \
            common_head(side, dt, SUBSYSTEM[arm]) + \
            ["  <asset>", f'    <material name="arm" rgba="{COLORS[arm]}"/>',
             f'    <material name="link" rgba="{COLORS["link"]}"/>']
        X += [f'    <mesh name="{n}" file="{fn}"/>' for n, fn in meshes]
        X += ["  </asset>", "  <worldbody>"] + lines + ["  </worldbody>"]
        X += arm_sections(side, arm, rename, loop_solref, loop_solimp)
        X.append("</mujoco>")
        files[f"arm_{arm}.xml"] = "\n".join(X) + "\n"

    X = header(side, "Top level: base + RightArm/LeftArm subsystems (arm_RA.xml, arm_LA.xml).") + \
        common_head(side, dt, "HumanoidUpperBody") + \
        ["  <asset>", f'    <material name="base" rgba="{COLORS["base"]}"/>']
    X += [f'    <mesh name="{n}" file="{fn}"/>' for n, fn in top_meshes]
    X += [f'    <model name="{a}" file="arm_{a}.xml"/>' for a in arms]
    X += ["  </asset>", "  <worldbody>"] + top_lines + ["  </worldbody>", "</mujoco>"]
    files["humanoid_upper.xml"] = "\n".join(X) + "\n"
    return files


def compile_string(xml: str) -> mj.MjModel:
    return mj.MjModel.from_xml_string(
        xml.replace('meshdir="../meshes"', f'meshdir="{(HERE / "meshes").as_posix()}"'))


def keyframes(side, m: mj.MjModel, with_ctrl: bool) -> list[str]:
    """assembly (q = 0) and rmd_ic (joint IC), by joint name.  Servo models also
    get ctrl = the key's own joint angles, so loading a key holds still."""
    q_ic = np.zeros(m.nq)
    for jn, v in side["initial_q"].items():
        q_ic[m.jnt_qposadr[m.joint(jn).id]] = v
    out = ["  <keyframe>"]
    for name, q, note in (("assembly", np.zeros(m.nq), ""),
                          ("rmd_ic", q_ic, "  <!-- joint IC displacements from the rmd -->")):
        ctrl = ""
        if with_ctrl:
            ctrl = f' ctrl="{f17([q[m.jnt_qposadr[m.actuator_trnid[i, 0]]] for i in range(m.nu)])}"'
        out.append(f'    <key name="{name}" qpos="{f17(q)}"{ctrl}/>{note}')
    out.append("  </keyframe>")
    return out


def add_keys(top: str, keys: list[str]) -> str:
    return top.replace("</mujoco>\n", "\n".join(keys) + "\n</mujoco>\n")


def write_files(files: dict[str, str]) -> None:
    OUT.mkdir(exist_ok=True)
    for name, text in files.items():
        (OUT / name).write_text(text, encoding="utf-8")


# --------------------------------------------------------------------------
def compare_models(a: mj.MjModel, b: mj.MjModel) -> tuple[float, list[str]]:
    """Name-by-name comparison of two compiled models.  Returns (max numeric
    difference, list of structural mismatches)."""
    bad: list[str] = []
    worst = 0.0

    def names(m, kind, n):
        return [getattr(m, kind)(i).name for i in range(n)]

    def diff(x, y):
        nonlocal worst
        worst = max(worst, float(np.abs(np.asarray(x, float) - np.asarray(y, float)).max(initial=0)))

    for kind, na, nb in (("body", a.nbody, b.nbody), ("joint", a.njnt, b.njnt),
                         ("geom", a.ngeom, b.ngeom), ("site", a.nsite, b.nsite),
                         ("equality", a.neq, b.neq), ("actuator", a.nu, b.nu),
                         ("sensor", a.nsensor, b.nsensor), ("key", a.nkey, b.nkey),
                         ("mesh", a.nmesh, b.nmesh)):
        sa, sb = set(names(a, kind, na)), set(names(b, kind, nb))
        if sa != sb:
            bad.append(f"{kind}: only in A {sorted(sa - sb)}, only in B {sorted(sb - sa)}")
    if bad:
        return worst, bad

    for i in range(a.nbody):
        k = b.body(a.body(i).name).id
        if a.body(a.body_parentid[i]).name != b.body(b.body_parentid[k]).name:
            bad.append(f"parent of {a.body(i).name}")
        for f in ("body_pos", "body_quat", "body_mass", "body_ipos", "body_iquat",
                  "body_inertia", "body_gravcomp"):
            diff(getattr(a, f)[i], getattr(b, f)[k])
    for i in range(a.njnt):
        k = b.joint(a.joint(i).name).id
        if a.jnt_type[i] != b.jnt_type[k] or \
           a.body(a.jnt_bodyid[i]).name != b.body(b.jnt_bodyid[k]).name:
            bad.append(f"joint {a.joint(i).name}")
        for f in ("jnt_axis", "jnt_pos", "dof_damping", "dof_armature", "dof_frictionloss"):
            src = getattr(a, f)
            ia, ib = (i, k) if f.startswith("jnt") else (a.jnt_dofadr[i], b.jnt_dofadr[k])
            diff(src[ia], getattr(b, f)[ib])
    for i in range(a.ngeom):
        k = b.geom(a.geom(i).name).id
        if a.body(a.geom_bodyid[i]).name != b.body(b.geom_bodyid[k]).name or \
           a.geom_contype[i] != b.geom_contype[k] or a.geom_conaffinity[i] != b.geom_conaffinity[k]:
            bad.append(f"geom {a.geom(i).name}")
        diff(a.geom_pos[i], b.geom_pos[k]); diff(a.geom_quat[i], b.geom_quat[k])
        diff(a.geom_rgba[i], b.geom_rgba[k])
        ma, mb = a.geom_dataid[i], b.geom_dataid[k]
        if a.mesh_vertnum[ma] != b.mesh_vertnum[mb] or a.mesh_facenum[ma] != b.mesh_facenum[mb]:
            bad.append(f"mesh of geom {a.geom(i).name}")
    for i in range(a.nsite):
        k = b.site(a.site(i).name).id
        if a.body(a.site_bodyid[i]).name != b.body(b.site_bodyid[k]).name:
            bad.append(f"site {a.site(i).name}")
        diff(a.site_pos[i], b.site_pos[k]); diff(a.site_quat[i], b.site_quat[k])
    for i in range(a.neq):
        k = b.equality(a.equality(i).name).id
        if a.eq_type[i] != b.eq_type[k] or \
           a.body(a.eq_obj1id[i]).name != b.body(b.eq_obj1id[k]).name or \
           a.body(a.eq_obj2id[i]).name != b.body(b.eq_obj2id[k]).name:
            bad.append(f"equality {a.equality(i).name}")
        diff(a.eq_data[i], b.eq_data[k]); diff(a.eq_solref[i], b.eq_solref[k])
        diff(a.eq_solimp[i], b.eq_solimp[k])
    for i in range(a.nu):
        k = b.actuator(a.actuator(i).name).id
        if a.joint(a.actuator_trnid[i, 0]).name != b.joint(b.actuator_trnid[k, 0]).name:
            bad.append(f"actuator {a.actuator(i).name}")
        for f in ("actuator_gear", "actuator_gainprm", "actuator_biasprm", "actuator_ctrlrange",
                  "actuator_gaintype", "actuator_biastype", "actuator_ctrllimited"):
            diff(getattr(a, f)[i], getattr(b, f)[k])
    for i in range(a.nsensor):
        k = b.sensor(a.sensor(i).name).id
        if a.sensor_type[i] != b.sensor_type[k] or a.sensor_dim[i] != b.sensor_dim[k]:
            bad.append(f"sensor {a.sensor(i).name}")
    for f in ("timestep", "gravity", "integrator", "disableflags", "enableflags"):
        diff(getattr(a.opt, f), getattr(b.opt, f))

    # keyframes and the resulting pose, per joint / body name
    da, db = mj.MjData(a), mj.MjData(b)
    for i in range(a.nkey):
        k = b.key(a.key(i).name).id
        mj.mj_resetDataKeyframe(a, da, i); mj.mj_resetDataKeyframe(b, db, k)
        mj.mj_forward(a, da); mj.mj_forward(b, db)
        for j in range(a.njnt):
            jb = b.joint(a.joint(j).name).id
            diff(da.qpos[a.jnt_qposadr[j]], db.qpos[b.jnt_qposadr[jb]])
        for u in range(a.nu):
            diff(da.ctrl[u], db.ctrl[b.actuator(a.actuator(u).name).id])
        for bi in range(a.nbody):
            kb = b.body(a.body(bi).name).id
            diff(da.xpos[bi], db.xpos[kb]); diff(da.xmat[bi], db.xmat[kb])
            diff(da.xipos[bi], db.xipos[kb])
    return worst, bad


def check_against_native(m: mj.MjModel) -> tuple[float, float]:
    """Compare with MuJoCo's own URDF import, body by body, in world frame at q=0."""
    n = mj.MjModel.from_xml_path(str(URDF))
    dm, dn = mj.MjData(m), mj.MjData(n)
    mj.mj_kinematics(m, dm)
    mj.mj_kinematics(n, dn)
    worst, worst_I = 0.0, 0.0
    for b in range(1, n.nbody):
        name = n.body(b).name
        if n.body_mass[b] == 0 and n.body_parentid[b] and name.endswith("_tool"):
            s = m.site(name).id
            worst = max(worst, np.abs(dn.xpos[b] - dm.site_xpos[s]).max(),
                        np.abs(dn.xmat[b] - dm.site_xmat[s]).max())
            continue
        k = m.body(name).id
        worst = max(worst,
                    np.abs(dn.xpos[b] - dm.xpos[k]).max(),
                    np.abs(dn.xmat[b] - dm.xmat[k]).max(),
                    np.abs(dn.xipos[b] - dm.xipos[k]).max(),
                    abs(n.body_mass[b] - m.body_mass[k]))
        if n.body_mass[b] > 0:          # world-frame tensors; native carries eig3 error
            In = dn.ximat[b].reshape(3, 3) @ np.diag(n.body_inertia[b]) @ dn.ximat[b].reshape(3, 3).T
            Im = dm.ximat[k].reshape(3, 3) @ np.diag(m.body_inertia[k]) @ dm.ximat[k].reshape(3, 3).T
            worst_I = max(worst_I, np.abs(In - Im).max() / np.abs(Im).max())
    for j in range(n.njnt):
        k = m.joint(n.joint(j).name).id
        worst = max(worst, np.abs(dn.xaxis[j] - dm.xaxis[k]).max(),
                    np.abs(dn.xanchor[j] - dm.xanchor[k]).max())
    return worst, worst_I


# --------------------------------------------------------------------------
def servo_files(files: dict[str, str], m: mj.MjModel, side: dict, hz: float,
                q6_margin_deg: float = 10.0) -> dict[str, str]:
    """Slider variant of the split files (humanoid_upper_servo.xml, arm_*_servo.xml).

    * every body gets gravcomp="1": MuJoCo applies -m*g at each CoM, including
      the linkage bodies, so gravity is cancelled exactly through the loop and
      the servos only have to track.  This force is NOT part of the actuator
      torque (see README);
    * RotationalAxial motors -> <position> servos, ctrl = target angle [rad];
      kp = M_ii * w^2, kv = 2 * sqrt(kp * M_ii) (critically damped), with M_ii the
      largest joint-space inertia seen at the two keyframes, so every joint gets
      about the same bandwidth;
    * q6 slider stops q6_margin short of the wrist parallelogram's singular pose.
    The torque-motor model stays the RecurDyn-faithful one.
    """
    w = 2 * np.pi * hz
    d = mj.MjData(m)
    Mii = np.zeros(m.nv)
    for k in range(m.nkey):
        mj.mj_resetDataKeyframe(m, d, k)
        mj.mj_forward(m, d)
        M = np.zeros((m.nv, m.nv))
        mj.mj_fullM(m, d, M)
        Mii = np.maximum(Mii, np.diag(M))

    # q6 drives the wrist parallelogram, which folds flat (singular) at some q6
    # on each side; past it the loop can snap to the crossed branch.
    sing = loops.q6_singular_limits(m)
    margin = np.radians(q6_margin_deg)

    out = {}
    for arm in sorted(side["tools"], reverse=True):
        acts = []
        for a in side["actuators"]:
            s_, jn = a["joint"].split("_", 1)
            if s_ != arm:
                continue
            dof = m.jnt_dofadr[m.joint(a["joint"]).id]
            kp = Mii[dof] * w * w
            kv = 2 * np.sqrt(kp * Mii[dof])
            lo, hi, note = -np.pi, np.pi, ""
            if jn == "joint6" and arm in sing:
                lo, hi = sing[arm][0] + margin, sing[arm][1] - margin
                note = (f"; linkage singular at {np.degrees(sing[arm][0]):.1f} / "
                        f"{np.degrees(sing[arm][1]):.1f} deg")
                print(f"{a['joint']}: parallelogram singular at {np.degrees(sing[arm][0]):+.2f} / "
                      f"{np.degrees(sing[arm][1]):+.2f} deg -> slider "
                      f"{np.degrees(lo):+.1f} .. {np.degrees(hi):+.1f} deg")
            local = a["name"].split("_", 1)[1].replace("motor", "servo")
            acts.append(f'    <position name="{local}" joint="{jn}" kp="{kp:.6g}" kv="{kv:.6g}" '
                        f'ctrlrange="{lo:.6f} {hi:.6f}"/>  <!-- M_ii {Mii[dof]:.3g} kg*m^2{note} -->')

        xml = files[f"arm_{arm}.xml"]
        head, rest = xml.split("  <actuator>\n", 1)
        _, tail = rest.split("  </actuator>\n", 1)
        xml = head + "  <actuator>\n" + "\n".join(acts) + "\n  </actuator>\n" + tail
        xml = xml.replace('name="motor', 'name="servo').replace('actuator="motor', 'actuator="servo')
        xml = xml.replace('<body name="', '<body gravcomp="1" name="')
        xml = xml.replace("Attached by humanoid_upper.xml", "Attached by humanoid_upper_servo.xml")
        xml = xml.replace("Do not edit by hand:",
                          f"Servo variant for the viewer sliders ({hz:g} Hz, gravcomp on).\n"
                          "     Do not edit by hand:")
        out[f"arm_{arm}_servo.xml"] = xml

    top = files["humanoid_upper.xml"]
    if "  <keyframe>\n" in top:                      # re-added with ctrl by build()
        a_, rest = top.split("  <keyframe>\n", 1)
        top = a_ + rest.split("  </keyframe>\n", 1)[1]
    top = top.replace('<body name="', '<body gravcomp="1" name="')
    top = top.replace('<mujoco model="HumanoidUpperBody">', '<mujoco model="HumanoidUpperBody servo">')
    for arm in side["tools"]:
        top = top.replace(f'file="arm_{arm}.xml"', f'file="arm_{arm}_servo.xml"')
    top = top.replace("(arm_RA.xml, arm_LA.xml)", "(arm_RA_servo.xml, arm_LA_servo.xml)")
    top = top.replace("Do not edit by hand:",
                      f"Servo variant for the viewer sliders ({hz:g} Hz, gravcomp on).\n"
                      "     Do not edit by hand:")
    out["humanoid_upper_servo.xml"] = top
    return out


def build(dt: float, loop_solref, loop_solimp, servo_hz: float = 4.0) -> mj.MjModel:
    side = json.loads(SIDECAR.read_text(encoding="utf-8"))

    # reference: the single-file model
    single = write_single(dt, loop_solref, loop_solimp)
    ref = compile_string(single)
    ref = compile_string(add_keys(single, keyframes(side, ref, False)))

    # the split files; keyframes need the compiled joint order, so two passes
    files = write_split(dt, loop_solref, loop_solimp)
    write_files(files)
    m = mj.MjModel.from_xml_path(str(OUT / "humanoid_upper.xml"))
    files["humanoid_upper.xml"] = add_keys(files["humanoid_upper.xml"], keyframes(side, m, False))
    files["scene.xml"] = make_scene(files["humanoid_upper.xml"])
    write_files(files)
    m = mj.MjModel.from_xml_path(str(OUT / "humanoid_upper.xml"))
    mj.MjModel.from_xml_path(str(OUT / "scene.xml"))
    for arm in sorted(side["tools"]):
        mj.MjModel.from_xml_path(str(OUT / f"arm_{arm}.xml"))       # opens on its own

    err, bad = compare_models(ref, m)
    print(f"subsystem split vs single-file model: max |diff| {err:.3e}, "
          f"structural mismatches {len(bad)}")
    if bad or err > 1e-12:
        raise RuntimeError(f"split model differs from the single-file model: {bad[:5]}")

    err, err_I = check_against_native(m)
    print(f"vs MuJoCo native URDF import: frames/mass/axes {err:.3e}, "
          f"inertia (relative) {err_I:.3e}")
    if err > 1e-12 or err_I > 1e-6:
        raise RuntimeError("MJCF disagrees with MuJoCo's own reading of the URDF")

    # slider variant, same layout
    sfiles = servo_files(files, m, side, servo_hz)
    write_files(sfiles)
    ms = mj.MjModel.from_xml_path(str(OUT / "humanoid_upper_servo.xml"))
    sfiles["humanoid_upper_servo.xml"] = add_keys(sfiles["humanoid_upper_servo.xml"],
                                                  keyframes(side, ms, True))
    sfiles["scene_servo.xml"] = make_scene(sfiles["humanoid_upper_servo.xml"])
    write_files(sfiles)
    ms = mj.MjModel.from_xml_path(str(OUT / "scene_servo.xml"))
    print(f"servo variant: nu {ms.nu}, {servo_hz:g} Hz, gravcomp on  -> mjcf/scene_servo.xml")
    return m


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dt", type=float, default=5e-4)
    ap.add_argument("--loop-solref", type=float, nargs=2, default=[0.002, 1.0])
    ap.add_argument("--loop-solimp", type=float, nargs=5, default=[0.99, 0.999, 1e-4, 0.5, 2])
    ap.add_argument("--servo-hz", type=float, default=4.0,
                    help="bandwidth of the slider servos in the viewer variant")
    a = ap.parse_args()
    m = build(a.dt, a.loop_solref, a.loop_solimp, a.servo_hz)
    print(f"MJCF  {OUT / 'humanoid_upper.xml'}  (+ arm_RA.xml, arm_LA.xml)")
    print(f"nbody {m.nbody}  nq {m.nq}  nu {m.nu}  neq {m.neq}  nkey {m.nkey}  "
          f"nsite {m.nsite}  nsensor {m.nsensor}  mass {m.body_subtreemass[0]:.4f} kg  "
          f"dt {m.opt.timestep}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
