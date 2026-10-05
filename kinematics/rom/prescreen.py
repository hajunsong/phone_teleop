"""Which solids could ever touch, per axis and direction.

RecurDyn's gap scope is exact but expensive -- about 23 s to create one on
imported CAD -- and this model carries 391 separate solids (191 on the torso
alone).  Creating a scope per solid pair is out of the question, so this
narrows the field first: sweep one joint with the other thirteen at their IC
and ask, per sample, which solid boxes overlap.  Only those pairs get a scope,
and the angle where a pair's boxes first meet is a lower bound on where its
real contact can begin -- which is what lets RomSweep stop early and still know
it has not missed a closer collision.

The test is box against box by separating axes, not axis-aligned extents: an
arm link's world-aligned box is mostly empty air, and the loose version calls
nearly everything a candidate.

Boxes come from ProcessNet (Geom.exe --solids-csv), stated in each subsystem's
own frame -- which is the frame our FK chain starts in (T_B0).  Kinematics come
from params/arm_{RA,LA}.json.  Nothing is hardcoded here.

    python prescreen.py [step_deg] [margin_mm]
"""
import csv, json, math, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
PARAMS = os.path.join(ROOT, "claude_ws", "params")

MM = 1e-3
SIDES = ["RA", "LA"]
SUB = {"RA": "RightArm", "LA": "LeftArm"}
# The wrist parallel-link bodies are lumped into real links the same way
# build_arm_params does it, so the prescreen sees the same seven-link arm.
RIDES_WITH = {"body6_1": 6, "body6_2": 5}

# body6_1 and body6_2 close a four-bar with body5 and body6 (RevJoint6_1/6_2/6_3).
# Lumping them onto single links -- 6_1 with body6, 6_2 with body5 -- is right
# for mass, but it freezes their motion relative to the rest of that loop: turn
# the wrist and 6_1 swings while 6_2 stays put, so they approach each other in a
# way the real linkage never does.  Pairs inside the loop are therefore not
# evidence of anything and are dropped here rather than misread as a limit.
# Against anything outside the loop they carry real geometry and stay.
LOOP = {"body5", "body6", "body6_1", "body6_2"}


def in_wrist_loop(a, b):
    return a in LOOP and b in LOOP and ("body6_1" in (a, b) or "body6_2" in (a, b))


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def fk_frames(arm, q):
    """A_i, r_i for i = 0..7 in the arm base frame (A_0 = I, r_0 = 0)."""
    A, r = [np.eye(3)], [np.zeros(3)]
    for i, lk in enumerate(arm["links"]):
        r.append(r[i] + A[i] @ np.array(lk["s"]))
        A.append(A[i] @ np.array(lk["C"]) @ rotz(q[i]))
    return A, r


class Solid(object):
    """A box: centre and half-extents in the frame of the link that carries it."""

    __slots__ = ("side", "link", "body", "full", "c", "h")

    def __init__(self, side, link, body, full, lo, hi):
        self.side, self.link, self.body, self.full = side, link, body, full
        self.c = (lo + hi) / 2.0
        self.h = (hi - lo) / 2.0


def load():
    arms = {s: json.load(open(os.path.join(PARAMS, "arm_%s.json" % s))) for s in SIDES}
    zero = {s: fk_frames(arms[s], np.zeros(7)) for s in SIDES}
    solids = []
    with open(os.path.join(HERE, "solid_boxes.csv"), encoding="utf-8-sig", newline="") as f:
        for row in csv.DictReader(f):
            lo = np.array([float(row[k]) for k in ("x1", "y1", "z1")]) * MM
            hi = np.array([float(row[k]) for k in ("x2", "y2", "z2")]) * MM
            sub, body = row["subsystem"].split("@")[0], row["body"]
            if sub == "HumanoidUpperBody":
                solids.append(Solid(None, 0, body, row["fullname"], lo, hi))
                continue
            side = "RA" if sub == "RightArm" else "LA"
            link = RIDES_WITH.get(body, int(body[4:]) if body.startswith("body") else None)
            if link is None:
                continue
            # Into the link's own frame, where it stays put as the joint turns.
            A0, r0 = zero[side]
            c = A0[link].T @ ((lo + hi) / 2.0 - r0[link])
            s = Solid(side, link, body, row["fullname"], lo, hi)
            s.c, s.h = c, (hi - lo) / 2.0          # extents are frame-aligned already
            solids.append(s)
    return arms, solids


def sat_overlap(cA, RA_, hA, cB, RB, hB, margin):
    """Separating-axis test, one box A against many boxes B.

    cB/RB/hB carry a leading axis over the B boxes; returns a bool per B.
    Fifteen axes: three from each box, nine cross products.
    """
    R = np.einsum("ij,njk->nik", RA_.T, RB)       # B's axes in A's frame
    absR = np.abs(R) + 1e-12
    t = np.einsum("ij,nj->ni", RA_.T, cB - cA)    # B's centre in A's frame

    hA_ = hA + margin
    hB_ = hB + margin
    # A's three axes, then B's three.
    sep = np.any(np.abs(t) > hA_ + np.einsum("nij,nj->ni", absR, hB_), axis=1)
    tB = np.einsum("nij,ni->nj", R, t)
    sep |= np.any(np.abs(tB) > np.einsum("nij,i->nj", absR, hA_) + hB_, axis=1)

    # The nine cross-product axes.  Skipping them would only ever call a pair
    # a candidate that is not one, so this stays sound if a term degenerates.
    for i in range(3):
        for j in range(3):
            i1, i2 = (i + 1) % 3, (i + 2) % 3
            j1, j2 = (j + 1) % 3, (j + 2) % 3
            ra = hA_[i1] * absR[:, i2, j] + hA_[i2] * absR[:, i1, j]
            rb = hB_[:, j1] * absR[:, i, j2] + hB_[:, j2] * absR[:, i, j1]
            d = np.abs(t[:, i2] * R[:, i1, j] - t[:, i1] * R[:, i2, j])
            sep |= d > ra + rb
    return ~sep


def main():
    step = math.radians(float(sys.argv[1]) if len(sys.argv) > 1 else 1.0)
    margin = (float(sys.argv[2]) if len(sys.argv) > 2 else 0.0) * MM
    arms, solids = load()
    sweep = np.arange(-math.pi, math.pi + 1e-9, step)

    def place(side, link, q):
        """Rotation and centre, in the model frame, of everything on that link."""
        R0, p0 = np.array(arms[side]["T_B0"]["R"]), np.array(arms[side]["T_B0"]["p"])
        A, r = fk_frames(arms[side], q)
        return R0 @ A[link], R0 @ r[link] + p0

    rows = []
    for side in SIDES:
        for j in range(1, 8):
            other = "LA" if side == "RA" else "RA"
            fixed = [s for s in solids
                     if s.side is None or s.side == other or (s.side == side and s.link < j)]
            fc, fR = [], []
            for s in fixed:
                if s.side is None:
                    fc.append(s.c)
                    fR.append(np.eye(3))
                else:
                    R, p = place(s.side, s.link, np.zeros(7))
                    fc.append(R @ s.c + p)
                    fR.append(R)
            fc, fR = np.array(fc), np.array(fR)
            fh = np.array([s.h for s in fixed])

            moving = [s for s in solids if s.side == side and s.link >= j]
            first = {}
            for qj in sweep:
                q = np.zeros(7)
                q[j - 1] = qj
                place_cache = {L: place(side, L, q) for L in range(j, 8)}
                for s in moving:
                    R, p = place_cache[s.link]
                    hit = sat_overlap(R @ s.c + p, R, s.h, fc, fR, fh, margin)
                    for k in np.nonzero(hit)[0]:
                        if in_wrist_loop(s.body, fixed[k].body):
                            continue
                        key = (s.body, s.full, fixed[k].side, fixed[k].body, fixed[k].full)
                        d = "+" if qj >= 0 else "-"
                        cur = first.get((key, d))
                        if cur is None or abs(qj) < abs(cur):
                            first[(key, d)] = qj

            for (key, d), qj in first.items():
                mb, ms, osd, ob, osn = key
                rows.append((side, j, d, mb, ms, osd or "HumanoidUpperBody", ob, osn,
                             round(math.degrees(qj), 2)))
            pos = [v for (k, d), v in first.items() if d == "+"]
            neg = [v for (k, d), v in first.items() if d == "-"]
            print("%s q%d: %4d pairs  (+ earliest %s, - earliest %s)" % (
                side, j, len(first),
                ("%.0f deg" % math.degrees(min(pos, key=abs))) if pos else "none",
                ("%.0f deg" % math.degrees(min(neg, key=abs))) if neg else "none"))

    rows.sort(key=lambda r: (r[0], r[1], r[2], abs(r[8])))
    out = os.path.join(HERE, "candidate_pairs.csv")
    with open(out, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["side", "axis", "dir", "moving_body", "moving_solid",
                    "other_sub", "other_body", "other_solid", "q_onset_deg"])
        w.writerows(rows)
    print("wrote %s (%d rows)" % (out, len(rows)))


if __name__ == "__main__":
    main()
