"""Run RomPose over the axes and collect the drive ranges.

(Copy of motion_generation/rom/run_pose.py for the 2026-10-06 re-measurement:
reads a frozen copy of the model and writes next to itself.)

The joint axis each sweep turns about is taken from the same FK the rest of the
codebase uses -- frame A_j's z direction through r_j, carried into the model
frame by T_B0 and converted to millimetres, which is what ObjectControl wants.

    python run_pose.py [--axes RA_q1,RA_q2] [--theta 180] [--tol 0.25]
                       [--margin 0]

Results land in rom_limits_pose.csv, one row per axis, appended as they finish
so a long run can be stopped and resumed.
"""
import csv, json, math, os, shutil, subprocess, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
PARAMS = os.path.join(ROOT, "claude_ws", "params")
ORIGINAL = os.path.join(HERE, "HumanoidUpperBody_20261006.rdyn")   # frozen copy of the 2026-10-06 02:02 save
COPY = os.path.join(HERE, "HumanoidUpperBody_rom.rdyn")
EXE = os.path.join(ROOT, "claude_ws", "motion_generation", "processnet", "RomPose.exe")
PAIRS = os.path.join(HERE, "candidate_pairs.csv")


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def joint_axis(side, j):
    """(origin in mm, unit direction) of joint j, in the model frame."""
    arm = json.load(open(os.path.join(PARAMS, "arm_%s.json" % side)))
    A, r = [np.eye(3)], [np.zeros(3)]
    for lk in arm["links"]:
        r.append(r[-1] + A[-1] @ np.array(lk["s"]))
        A.append(A[-1] @ np.array(lk["C"]) @ rotz(0.0))
    R0 = np.array(arm["T_B0"]["R"])
    p0 = np.array(arm["T_B0"]["p"])
    return (R0 @ r[j] + p0) * 1e3, R0 @ A[j][:, 2]


def moving_bodies(j):
    """What rides on joint j.  The wrist parallel links go with the bodies
    build_arm_params lumps them into: 6_1 with body6, 6_2 with body5."""
    out = ["body%d" % i for i in range(j, 8)]
    if j <= 6:
        out.append("body6_1")
    if j <= 5:
        out.append("body6_2")
    return ",".join(out)


def opt(name, default):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else default


def main():
    axes = opt("--axes", ",".join("RA_q%d" % j for j in range(1, 8))).split(",")
    theta = opt("--theta", "180")
    tol = opt("--tol", "0.5")
    coarse = opt("--coarse", "15")
    margin = opt("--margin", "0")

    out = os.path.join(HERE, "rom_limits_pose.csv")
    done = {}
    if os.path.exists(out):
        with open(out, encoding="utf-8-sig", newline="") as f:
            done = {r["axis"]: r for r in csv.DictReader(f)}

    for axis in axes:
        if axis in done:
            print("%s: already done" % axis)
            continue
        side, j = axis[:2], int(axis[-1])
        org, dirv = joint_axis(side, j)
        # An interrupted run leaves its copy open in RecurDyn, and an open
        # document locks the file against being overwritten.
        subprocess.call([EXE, COPY, "--close-only"],
                        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        shutil.copyfile(ORIGINAL, COPY)
        gap = os.path.join(HERE, "pose_%s.csv" % axis)
        log = os.path.join(HERE, "pose_%s.log" % axis)
        cmd = [EXE, COPY, "--pairs", PAIRS, "--axis", axis,
               "--origin", "%.6f,%.6f,%.6f" % tuple(org),
               "--dir", "%.6f,%.6f,%.6f" % tuple(dirv),
               "--bodies", moving_bodies(j),
               "--theta", theta, "--coarse", coarse, "--tol", tol,
               "--margin", margin, "--out", gap]
        t0 = time.time()
        with open(log, "w", encoding="utf-8") as f:
            rc = subprocess.call(cmd, stdout=f, stderr=subprocess.STDOUT)
        print("%s: rc=%d in %.0f s" % (axis, rc, time.time() - t0))
        if rc != 0 or not os.path.exists(gap):
            print("  no result; see " + log)
            continue

        lim = {"+": None, "-": None}
        who = {"+": None, "-": None}
        with open(gap, encoding="utf-8-sig", newline="") as f:
            for r in csv.DictReader(f):
                if not r["limit_deg"]:
                    continue
                lim[r["dir"]] = float(r["limit_deg"])
                who[r["dir"]] = r["moving_solid"] + " vs " + r["other_solid"]
        done[axis] = {"axis": axis,
                      "q_min_deg": "" if lim["-"] is None else "%.2f" % lim["-"],
                      "q_max_deg": "" if lim["+"] is None else "%.2f" % lim["+"],
                      "blocks_min": who["-"] or "", "blocks_max": who["+"] or ""}
        with open(out, "w", encoding="utf-8", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["axis", "q_min_deg", "q_max_deg",
                                              "blocks_min", "blocks_max"])
            w.writeheader()
            for a in done:
                w.writerow(done[a])
        print("  %s: [%s, %s]" % (axis, done[axis]["q_min_deg"] or "clear",
                                  done[axis]["q_max_deg"] or "clear"))
    print("wrote " + out)


if __name__ == "__main__":
    main()
