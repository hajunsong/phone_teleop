package com.humanoid.teleop;

import java.io.FileInputStream;
import java.util.Locale;

/** Desktop check of Controller (run by tools/check_phone_kinematics.py --controller). */
public class ControllerCheck {
    static RobotModel m;
    static RobotModel.Ik ik;
    static int fails = 0;

    static double[] tool(Controller c, int s) {
        double[] p = new double[3], R = new double[9];
        ik.fk(s, c.q[s].clone(), p, R);
        return p;
    }

    static void run(Controller c, double sec, double[] maxStep) {
        double[][] prev = c.snapshotQ();
        for (int i = 0; i < (int) Math.round(sec * 100); i++) {
            c.tick(0.01);
            for (int s = 0; s < 2; s++)
                for (int k = 0; k < 7; k++) {
                    maxStep[0] = Math.max(maxStep[0], Math.abs(c.q[s][k] - prev[s][k]));
                    prev[s][k] = c.q[s][k];
                }
        }
    }

    static void check(String what, boolean ok, String detail) {
        System.out.println((ok ? "  ok   " : "  FAIL ") + what + "  " + detail);
        if (!ok) fails++;
    }

    public static void main(String[] a) throws Exception {
        m = RobotModel.load(new FileInputStream(a[0]));
        ik = m.new Ik();
        double[] ms = {0};
        Controller c = new Controller(m);
        c.speed01 = 0.5;                                        // 77.5 mm/s, 32.5 deg/s
        int RA = 0, LA = 1;

        // reach probe: from home, hold each base-frame button 4 s; report travel along the
        // asked axis and the worst drift on the other axes (must stay ~0)
        String[] ax = {"X", "Y", "Z", "Rx", "Ry", "Rz"};
        double worstDrift = 0, worstRotDrift = 0;
        for (int s = 0; s < 2; s++) {
            StringBuilder sb = new StringBuilder("  reach from home " + RobotModel.SIDE[s] + ":");
            for (int k = 0; k < 6; k++)
                for (int dir = -1; dir <= 1; dir += 2) {
                    Controller cc = new Controller(m);
                    cc.speed01 = 0.5;
                    double[] a0 = new double[3], A0 = new double[9], a1 = new double[3], A1 = new double[9];
                    ik.fk(s, cc.q[s].clone(), a0, A0);
                    cc.setHold(s, k, dir);
                    run(cc, 4.0, ms);
                    ik.fk(s, cc.q[s].clone(), a1, A1);
                    double[] A0t = new double[9], dRm = new double[9];
                    RobotModel.transpose(A0, A0t);
                    RobotModel.mul(A1, A0t, dRm);
                    double[] w = RobotModel.rotvec(dRm);              // world-frame rotation
                    double along, drift = 0, rdrift = 0;
                    if (k < 3) {
                        along = (a1[k] - a0[k]) * 1e3 * dir;
                        for (int i = 0; i < 3; i++) if (i != k) drift = Math.max(drift, Math.abs(a1[i] - a0[i]));
                        rdrift = Math.sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
                    } else {
                        along = Math.toDegrees(w[k - 3]) * dir;
                        for (int i = 0; i < 3; i++) if (i != k - 3) rdrift = Math.max(rdrift, Math.abs(w[i]));
                        for (int i = 0; i < 3; i++) drift = Math.max(drift, Math.abs(a1[i] - a0[i]));
                    }
                    if (drift > 2e-3 || rdrift > Math.toRadians(1)) System.out.println(String.format(Locale.US, "    drift %s %s%s: %.1f mm %.2f deg", RobotModel.SIDE[s], ax[k], dir > 0 ? "+" : "-", drift * 1e3, Math.toDegrees(rdrift)));
                    worstDrift = Math.max(worstDrift, drift);
                    worstRotDrift = Math.max(worstRotDrift, rdrift);
                    sb.append(String.format(Locale.US, " %s%s %.0f%s", ax[k], dir > 0 ? "+" : "-", along, k < 3 ? "mm" : "°"));
                }
            System.out.println(sb);
        }
        check("jog moves only along the asked axis (rotation may yield <= 30 deg on translations)", worstDrift < 2e-3 && worstRotDrift < Math.toRadians(30.6),
                String.format(Locale.US, "worst off-axis drift %.2f mm, %.2f deg",
                        worstDrift * 1e3, Math.toDegrees(worstRotDrift)));

        double[] p0 = tool(c, LA);
        c.setHold(LA, 1, +1);
        run(c, 1.0, ms);
        c.setHold(LA, 1, 0);
        run(c, 0.5, ms);
        double[] p1 = tool(c, LA);
        check("continuous LA Y+ 1 s", Math.abs((p1[1] - p0[1]) - 0.0775) < 0.002,
                String.format(Locale.US, "dy %.1f mm (expect 77.5)", (p1[1] - p0[1]) * 1e3));

        c.continuous = false;
        c.mirror = true;
        double[] l0 = tool(c, LA), r0 = tool(c, RA);
        c.step(LA, 1, +1, 0.02);                                // LA Y+ 20 mm, mirrored to RA Y-
        run(c, 1.0, ms);
        double[] l1 = tool(c, LA), r1 = tool(c, RA);
        check("step + mirror", Math.abs(l1[1] - l0[1] - 0.02) < 5e-4 && Math.abs(r1[1] - r0[1] + 0.02) < 5e-4,
                String.format(Locale.US, "LA dy %+.1f mm, RA dy %+.1f mm", (l1[1] - l0[1]) * 1e3, (r1[1] - r0[1]) * 1e3));
        c.mirror = false;

        c.continuous = true;
        c.setHold(RA, 0, +1);                                   // push RA forward far beyond reach
        run(c, 8.0, ms);
        String st = c.state[RA];
        double[] pa = tool(c, RA);
        c.setHold(RA, 0, -1);                                   // back off: must respond at once
        run(c, 0.2, ms);
        c.setHold(RA, 0, 0);
        double[] pb = tool(c, RA);
        check("limit, no wind-up", st.equals("한계") && pb[0] < pa[0] - 0.005,
                String.format(Locale.US, "state '%s' at x %.3f, after 0.2 s back x %.3f", st, pa[0], pb[0]));

        double maxJ = c.jointSpeed() * 0.01;
        ms[0] = 0;
        c.moveHome(-1);
        run(c, 6.0, ms);
        double dq = 0;
        for (int s = 0; s < 2; s++) for (int k = 0; k < 7; k++) dq = Math.max(dq, Math.abs(c.q[s][k] - m.home[s][k]));
        check("home move continuous", ms[0] <= maxJ * 1.0001 && dq < 1e-9,
                String.format(Locale.US, "max step %.3f deg/tick (limit %.3f), final err %.1e rad",
                        Math.toDegrees(ms[0]), Math.toDegrees(maxJ), dq));

        c.toolFrame = true;
        double[] R0 = new double[9], R1 = new double[9], pp = new double[3];
        ik.fk(RA, c.q[RA].clone(), pp, R0);
        c.continuous = false;
        c.step(RA, 5, +1, Math.toRadians(10));                  // Rz+ 10 deg about the tool z
        run(c, 1.0, ms);
        ik.fk(RA, c.q[RA].clone(), pp, R1);
        double[] R0t = new double[9], d = new double[9];
        RobotModel.transpose(R0, R0t);
        RobotModel.mul(R0t, R1, d);                             // relative rotation in tool frame
        double[] w = RobotModel.rotvec(d);
        check("tool-frame Rz+ 10 deg", Math.abs(Math.toDegrees(w[2]) - 10) < 0.1 && Math.abs(w[0]) + Math.abs(w[1]) < 1e-3,
                String.format(Locale.US, "rotation in tool frame [%.2f %.2f %.2f] deg",
                        Math.toDegrees(w[0]), Math.toDegrees(w[1]), Math.toDegrees(w[2])));

        System.out.println(fails == 0 ? "PASS" : "FAIL");
        System.exit(fails == 0 ? 0 : 1);
    }
}
