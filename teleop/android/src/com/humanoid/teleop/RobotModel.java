package com.humanoid.teleop;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

/**
 * Kinematics of the upper body, read from assets/robot_model.txt (exported from
 * the MuJoCo model, which is generated from the RecurDyn rmd).  Plain Java, no
 * Android classes, so it can be checked against MuJoCo on a PC.
 *
 * Arms are indexed RA = 0, LA = 1.  Every joint rotates about its body's local
 * +z at the body origin.  Rotations are row-major double[9].
 */
public final class RobotModel {

    public static final int RA = 0, LA = 1;
    public static final String[] SIDE = {"RA", "LA"};

    public static final class Body {
        int parent;
        double[] pos = new double[3];
        double[] rot = new double[9];      // relative to parent, at joint angle 0
        int kind;                          // 0 fixed, 1 arm joint, 2 passive
        int side = -1, k = -1;
        double coef;
    }

    public static final class Geom {
        public int body, mesh;
        public double[] pos = new double[3];
        public double[] rot = new double[9];
        public float[] rgb = new float[3];
    }

    public final List<Body> bodies = new ArrayList<>();   // index = MuJoCo body id (0 = world)
    public final List<Geom> geoms = new ArrayList<>();
    public final int[] toolBody = new int[2];
    public final double[][] toolPos = new double[2][3];
    public final double[][] toolRot = new double[2][9];
    public final double[][] home = new double[2][7];
    public final double[][] qmin = new double[2][7];
    public final double[][] qmax = new double[2][7];
    final int[][] jointBody = new int[2][7];

    public static RobotModel load(InputStream in) throws IOException {
        RobotModel m = new RobotModel();
        m.bodies.add(new Body());                          // world
        m.bodies.get(0).rot = eye();
        BufferedReader r = new BufferedReader(new InputStreamReader(in, StandardCharsets.UTF_8));
        String line;
        while ((line = r.readLine()) != null) {
            line = line.trim();
            if (line.isEmpty() || line.startsWith("#")) continue;
            String[] t = line.split("\\s+");
            switch (t[0]) {
                case "body": {
                    Body b = new Body();
                    int id = Integer.parseInt(t[1]);
                    b.parent = Integer.parseInt(t[2]);
                    for (int i = 0; i < 3; i++) b.pos[i] = d(t[3 + i]);
                    b.rot = quatToRot(d(t[6]), d(t[7]), d(t[8]), d(t[9]));
                    b.kind = t[10].equals("arm") ? 1 : t[10].equals("passive") ? 2 : 0;
                    if (b.kind != 0) {
                        b.side = side(t[11]);
                        b.k = Integer.parseInt(t[12]);
                        b.coef = d(t[13]);
                    }
                    if (id != m.bodies.size()) throw new IOException("body ids out of order at " + id);
                    m.bodies.add(b);
                    if (b.kind == 1) m.jointBody[b.side][b.k] = id;
                    break;
                }
                case "geom": {
                    Geom g = new Geom();
                    g.body = Integer.parseInt(t[1]);
                    g.mesh = Integer.parseInt(t[2]);
                    for (int i = 0; i < 3; i++) g.pos[i] = d(t[3 + i]);
                    g.rot = quatToRot(d(t[6]), d(t[7]), d(t[8]), d(t[9]));
                    for (int i = 0; i < 3; i++) g.rgb[i] = (float) d(t[10 + i]);
                    m.geoms.add(g);
                    break;
                }
                case "arm": {
                    int s = side(t[1]);
                    m.toolBody[s] = Integer.parseInt(t[2]);
                    for (int i = 0; i < 3; i++) m.toolPos[s][i] = d(t[3 + i]);
                    m.toolRot[s] = quatToRot(d(t[6]), d(t[7]), d(t[8]), d(t[9]));
                    break;
                }
                case "home":
                case "qmin":
                case "qmax": {
                    double[][] dst = t[0].equals("home") ? m.home : t[0].equals("qmin") ? m.qmin : m.qmax;
                    int s = side(t[1]);
                    for (int i = 0; i < 7; i++) dst[s][i] = d(t[2 + i]);
                    break;
                }
                default:
                    break;
            }
        }
        return m;
    }

    // ------------------------------------------------------------------ kinematics

    /** World pose of every body.  q[side][7].  Writes pos[n][3], rot[n][9]. */
    public void fkAll(double[][] q, double[][] pos, double[][] rot) {
        pos[0] = new double[]{0, 0, 0};
        rot[0] = eye();
        double[] rz = new double[9], tmp = new double[9];
        for (int i = 1; i < bodies.size(); i++) {
            Body b = bodies.get(i);
            double[] pr = rot[b.parent], pp = pos[b.parent];
            double[] p = pos[i] == null ? (pos[i] = new double[3]) : pos[i];
            double[] r = rot[i] == null ? (rot[i] = new double[9]) : rot[i];
            mulv(pr, b.pos, p);
            p[0] += pp[0];
            p[1] += pp[1];
            p[2] += pp[2];
            mul(pr, b.rot, tmp);
            if (b.kind == 0) {
                System.arraycopy(tmp, 0, r, 0, 9);
            } else {
                double a = b.kind == 1 ? q[b.side][b.k] : b.coef * q[b.side][b.k];
                rotz(a, rz);
                mul(tmp, rz, r);
            }
        }
    }

    /** Tool pose of one arm: p[3], R[9] (world). */
    public void tool(int side, double[][] pos, double[][] rot, double[] p, double[] R) {
        int b = toolBody[side];
        mulv(rot[b], toolPos[side], p);
        p[0] += pos[b][0];
        p[1] += pos[b][1];
        p[2] += pos[b][2];
        mul(rot[b], toolRot[side], R);
    }

    /** IK workspace (one per thread). */
    public final class Ik {
        final double[][] pos = new double[bodies.size()][], rot = new double[bodies.size()][];
        final double[][] qq = new double[2][7];
        final double[] p = new double[3], R = new double[9];
        final double[] J = new double[6 * 7], e = new double[6], A = new double[36], y = new double[6];
        public double errPos, errRot;

        public void fk(int side, double[] q7, double[] pOut, double[] ROut) {
            qq[side] = q7;
            qq[1 - side] = home[1 - side];
            fkAll(qq, pos, rot);
            tool(side, pos, rot, pOut, ROut);
        }

        /**
         * Damped least squares toward (pDes, RDes), starting from and writing to q.
         * wRot [m/rad] weighs rotation against position, lam is the damping and
         * dqMax caps each iteration's step norm.  Joint limits are enforced.
         */
        public void solve(int side, double[] q, double[] pDes, double[] RDes,
                          int iters, double wRot, double lam, double dqMax) {
            double[] lo = qmin[side], hi = qmax[side];
            double[] dR = new double[9], RT = new double[9];
            for (int it = 0; it <= iters; it++) {
                fk(side, q, p, R);
                transpose(R, RT);
                mul(RDes, RT, dR);
                double[] w = rotvec(dR);
                e[0] = pDes[0] - p[0];
                e[1] = pDes[1] - p[1];
                e[2] = pDes[2] - p[2];
                errPos = Math.sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
                errRot = Math.sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
                if (it == iters || (errPos < 1e-5 && errRot < 1e-4)) break;
                e[3] = wRot * w[0];
                e[4] = wRot * w[1];
                e[5] = wRot * w[2];
                for (int k = 0; k < 7; k++) {
                    int jb = jointBody[side][k];
                    double[] o = pos[jb], rr = rot[jb];
                    double zx = rr[2], zy = rr[5], zz = rr[8];          // body z axis in world
                    double rx = p[0] - o[0], ry = p[1] - o[1], rzz = p[2] - o[2];
                    J[0 * 7 + k] = zy * rzz - zz * ry;
                    J[1 * 7 + k] = zz * rx - zx * rzz;
                    J[2 * 7 + k] = zx * ry - zy * rx;
                    J[3 * 7 + k] = wRot * zx;
                    J[4 * 7 + k] = wRot * zy;
                    J[5 * 7 + k] = wRot * zz;
                }
                // DLS with limit clamping: a joint sitting on a limit that the step would
                // push further out is locked (its column removed) and the step re-solved,
                // so the other joints take over instead of the solution stalling.
                boolean[] locked = new boolean[7];
                double[] dq = new double[7];
                double n2 = 0;
                for (int pass = 0; pass < 7; pass++) {
                    // A = J J^T + lam^2 I ;  y = A^-1 e ;  dq = J^T y   (locked columns excluded)
                    for (int i = 0; i < 6; i++)
                        for (int j = 0; j < 6; j++) {
                            double s = 0;
                            for (int k = 0; k < 7; k++) if (!locked[k]) s += J[i * 7 + k] * J[j * 7 + k];
                            A[i * 6 + j] = s + (i == j ? lam * lam : 0);
                        }
                    solve6(A, e, y);
                    n2 = 0;
                    for (int k = 0; k < 7; k++) {
                        double s = 0;
                        if (!locked[k]) for (int i = 0; i < 6; i++) s += J[i * 7 + k] * y[i];
                        dq[k] = s;
                        n2 += s * s;
                    }
                    boolean again = false;
                    for (int k = 0; k < 7; k++) {
                        if (locked[k]) continue;
                        if ((q[k] <= lo[k] + 1e-9 && dq[k] < 0) || (q[k] >= hi[k] - 1e-9 && dq[k] > 0)) {
                            locked[k] = true;
                            again = true;
                        }
                    }
                    if (!again) break;
                }
                double n = Math.sqrt(n2), sc = n > dqMax ? dqMax / n : 1.0;
                for (int k = 0; k < 7; k++) {
                    q[k] = Math.min(hi[k], Math.max(lo[k], q[k] + sc * dq[k]));
                }
            }
        }
    }

    // ------------------------------------------------------------------ small math

    static double d(String s) {
        return Double.parseDouble(s);
    }

    static int side(String s) {
        return s.equals("LA") ? LA : RA;
    }

    public static double[] eye() {
        return new double[]{1, 0, 0, 0, 1, 0, 0, 0, 1};
    }

    public static double[] quatToRot(double w, double x, double y, double z) {
        double n = Math.sqrt(w * w + x * x + y * y + z * z);
        w /= n; x /= n; y /= n; z /= n;
        return new double[]{
                1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
                2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
                2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)};
    }

    static void rotz(double a, double[] o) {
        double c = Math.cos(a), s = Math.sin(a);
        o[0] = c; o[1] = -s; o[2] = 0;
        o[3] = s; o[4] = c; o[5] = 0;
        o[6] = 0; o[7] = 0; o[8] = 1;
    }

    public static void mul(double[] a, double[] b, double[] o) {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                o[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
    }

    public static void mulv(double[] a, double[] v, double[] o) {
        double x = v[0], y = v[1], z = v[2];
        o[0] = a[0] * x + a[1] * y + a[2] * z;
        o[1] = a[3] * x + a[4] * y + a[5] * z;
        o[2] = a[6] * x + a[7] * y + a[8] * z;
    }

    public static void transpose(double[] a, double[] o) {
        o[0] = a[0]; o[1] = a[3]; o[2] = a[6];
        o[3] = a[1]; o[4] = a[4]; o[5] = a[7];
        o[6] = a[2]; o[7] = a[5]; o[8] = a[8];
    }

    /** log map SO(3) -> axis * angle. */
    public static double[] rotvec(double[] R) {
        double c = Math.max(-1, Math.min(1, (R[0] + R[4] + R[8] - 1) / 2));
        double th = Math.acos(c);
        double vx = R[7] - R[5], vy = R[2] - R[6], vz = R[3] - R[1];
        if (th < 1e-6) return new double[]{0.5 * vx, 0.5 * vy, 0.5 * vz};
        if (th > Math.PI - 1e-4) {
            double[] dg = {(R[0] + 1) / 2, (R[4] + 1) / 2, (R[8] + 1) / 2};
            int k = dg[0] >= dg[1] && dg[0] >= dg[2] ? 0 : dg[1] >= dg[2] ? 1 : 2;
            double s = Math.sqrt(Math.max(dg[k], 1e-12));
            double[] ax = new double[3];
            for (int i = 0; i < 3; i++) ax[i] = (R[i * 3 + k] + (i == k ? 1 : 0)) / 2 / s;
            return new double[]{ax[0] * th, ax[1] * th, ax[2] * th};
        }
        double f = th / (2 * Math.sin(th));
        return new double[]{f * vx, f * vy, f * vz};
    }

    /** exp map axis * angle -> SO(3). */
    public static double[] expmap(double wx, double wy, double wz) {
        double th = Math.sqrt(wx * wx + wy * wy + wz * wz);
        if (th < 1e-12) return eye();
        double x = wx / th, y = wy / th, z = wz / th, c = Math.cos(th), s = Math.sin(th), v = 1 - c;
        return new double[]{
                c + x * x * v, x * y * v - z * s, x * z * v + y * s,
                y * x * v + z * s, c + y * y * v, y * z * v - x * s,
                z * x * v - y * s, z * y * v + x * s, c + z * z * v};
    }

    /** Solve 6x6 SPD system by Gaussian elimination with partial pivoting (A is overwritten). */
    static void solve6(double[] A, double[] b, double[] x) {
        double[] bb = b.clone();
        for (int c = 0; c < 6; c++) {
            int piv = c;
            for (int r = c + 1; r < 6; r++) if (Math.abs(A[r * 6 + c]) > Math.abs(A[piv * 6 + c])) piv = r;
            if (piv != c) {
                for (int k = 0; k < 6; k++) {
                    double t = A[c * 6 + k]; A[c * 6 + k] = A[piv * 6 + k]; A[piv * 6 + k] = t;
                }
                double t = bb[c]; bb[c] = bb[piv]; bb[piv] = t;
            }
            for (int r = c + 1; r < 6; r++) {
                double f = A[r * 6 + c] / A[c * 6 + c];
                for (int k = c; k < 6; k++) A[r * 6 + k] -= f * A[c * 6 + k];
                bb[r] -= f * bb[c];
            }
        }
        for (int r = 5; r >= 0; r--) {
            double s = bb[r];
            for (int k = r + 1; k < 6; k++) s -= A[r * 6 + k] * x[k];
            x[r] = s / A[r * 6 + r];
        }
    }
}
