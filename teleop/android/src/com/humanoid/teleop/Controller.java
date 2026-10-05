package com.humanoid.teleop;

/**
 * Jog logic and IK for both arms.  Plain Java (no Android), driven by tick(dt)
 * at 100 Hz from a background thread; UI calls the input methods.  All public
 * methods are synchronized.
 *
 * Per arm there is a goal pose (where the operator asked to go) and a target
 * pose that chases the goal at the jog speed; the IK follows the target from
 * the current q, so every command is continuous:
 *   - continuous mode : a held button moves the goal at the jog speed
 *   - step mode       : a tap moves the goal by one step (mm or deg)
 *   - pad home        : goal position (or orientation) back to the home pose's
 *   - presets / home  : joint-space move to a stored q at a joint speed limit
 * When the IK cannot reach the target (limit / workspace edge), goal and target
 * snap back to what the arm actually reached, so nothing winds up.
 *
 * Frames: base (x forward, y left, z up) or tool.  "Mirror" applies an input on
 * one arm to both, reflected about the robot's sagittal plane (y -> -y).
 */
final class Controller {

    static final int RA = RobotModel.RA, LA = RobotModel.LA;
    static final double REACH_TOL_M = 0.001, REACH_TOL_RAD = Math.toRadians(0.5);   // per tick

    final RobotModel m;
    private final RobotModel.Ik ik;

    final double[][] q = new double[2][7];
    private final double[][] pT = new double[2][3], RT = new double[2][9];   // target
    private final double[][] pG = new double[2][3], RG = new double[2][9];   // goal
    private final double[][] pHome = new double[2][3], RHome = new double[2][9];
    private final double[][] qGoal = new double[2][];                         // joint move
    private final int[][] hold = new int[2][6];
    final String[] state = {"대기", "대기"};
    final boolean[] atLimit = new boolean[2];
    private final double[] limitHold = new double[2];   // keeps "한계" visible while pushing against it
    final double[] errPos = new double[2], errRot = new double[2];

    boolean continuous = true, toolFrame = false, mirror = false;
    /** Translation blocked with the tool orientation held -> let the orientation give way. */
    boolean positionPriority = true;
    final boolean[] yielded = new boolean[2];
    static final double YIELD_MAX_RAD = Math.toRadians(30);
    private final double[][] yieldAnchor = new double[2][];        // orientation when yielding began (per push)
    double speed01 = 0.5;

    Controller(RobotModel m) {
        this.m = m;
        this.ik = m.new Ik();
        for (int s = 0; s < 2; s++) {
            System.arraycopy(m.home[s], 0, q[s], 0, 7);
            ik.fk(s, m.home[s].clone(), pHome[s], RHome[s]);
        }
        resync();
    }

    // ------------------------------------------------------------------ speeds

    double linSpeed() {            // m/s
        return 0.005 + 0.145 * speed01;
    }

    double angSpeed() {            // rad/s
        return Math.toRadians(5 + 55 * speed01);
    }

    double jointSpeed() {          // rad/s, presets and home moves
        return Math.toRadians(10 + 50 * speed01);
    }

    // ------------------------------------------------------------------ inputs

    synchronized void setHold(int side, int axis, int dir) {
        hold[side][axis] = dir;
    }

    synchronized void releaseAll() {
        for (int[] h : hold) java.util.Arrays.fill(h, 0);
    }

    /** One step on one axis (step mode).  axis 0..2 position [m], 3..5 rotation [rad]. */
    synchronized void step(int side, int axis, int dir, double amount) {
        double[] v = new double[3], w = new double[3];
        if (axis < 3) v[axis] = dir * amount;
        else w[axis - 3] = dir * amount;
        applyDelta(side, v, w);
    }

    /** Pad home: bring the goal position (rot = false) or orientation (rot = true) to the home pose's. */
    synchronized void padHome(int side, boolean rot) {
        for (int s : affected(side)) {
            qGoal[s] = null;
            if (rot) System.arraycopy(RHome[s], 0, RG[s], 0, 9);
            else System.arraycopy(pHome[s], 0, pG[s], 0, 3);
        }
    }

    /** Joint-space move of one arm (or both with side = -1) to q. */
    synchronized void moveJoints(int side, double[] target) {
        int[] arms = side < 0 ? new int[]{RA, LA} : new int[]{side};
        for (int s : arms) qGoal[s] = clampQ(s, side < 0 ? m.home[s] : target);
    }

    synchronized void moveHome(int side) {
        if (side < 0) moveJoints(-1, null);
        else moveJoints(side, m.home[side]);
    }

    /** Take q from the robot (joint_state) and restart all motion from there. */
    synchronized void syncFrom(double[][] qRobot) {
        for (int s = 0; s < 2; s++) {
            double[] c = clampQ(s, qRobot[s]);
            System.arraycopy(c, 0, q[s], 0, 7);
            qGoal[s] = null;
        }
        releaseAll();
        resync();
    }

    synchronized double[][] snapshotQ() {
        return new double[][]{q[RA].clone(), q[LA].clone()};
    }

    synchronized boolean busy(int side) {
        if (qGoal[side] != null) return true;
        for (int v : hold[side]) if (v != 0) return true;
        return dist(pT[side], pG[side]) > 1e-6 || angle(RT[side], RG[side]) > 1e-6;
    }

    // ------------------------------------------------------------------ tick

    synchronized void tick(double dt) {
        double lin = linSpeed(), ang = angSpeed();
        // continuous jog: held buttons move the goal
        if (continuous) {
            for (int src = 0; src < 2; src++) {
                int[] h = hold[src];
                boolean any = false;
                for (int v : h) any |= v != 0;
                if (!any) continue;
                double[] v = {h[0] * lin * dt, h[1] * lin * dt, h[2] * lin * dt};
                double[] w = {h[3] * ang * dt, h[4] * ang * dt, h[5] * ang * dt};
                applyDelta(src, v, w);
            }
        }
        for (int s = 0; s < 2; s++) {
            if (qGoal[s] != null) {
                tickJoint(s, dt);
                continue;
            }
            // target chases goal at the jog speed
            double[] dp = {pG[s][0] - pT[s][0], pG[s][1] - pT[s][1], pG[s][2] - pT[s][2]};
            double n = Math.sqrt(dp[0] * dp[0] + dp[1] * dp[1] + dp[2] * dp[2]);
            double[] RtT = new double[9], dR = new double[9];
            RobotModel.transpose(RT[s], RtT);
            RobotModel.mul(RG[s], RtT, dR);
            double[] w = RobotModel.rotvec(dR);
            double a = Math.sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
            limitHold[s] = Math.max(0, limitHold[s] - dt);
            if (n < 1e-7 && a < 1e-7) {
                yieldAnchor[s] = null;                              // motion ended: new yield budget
                atLimit[s] = limitHold[s] > 0;
                state[s] = atLimit[s] ? "한계" : "대기";
                continue;
            }
            double[] pPrev = pT[s].clone(), RPrev = RT[s].clone();
            double fp = n > lin * dt ? lin * dt / n : 1, fa = a > ang * dt ? ang * dt / a : 1;
            for (int i = 0; i < 3; i++) pT[s][i] += fp * dp[i];
            double[] Rs = RobotModel.expmap(fa * w[0], fa * w[1], fa * w[2]), Rn = new double[9];
            RobotModel.mul(Rs, RT[s], Rn);
            RT[s] = Rn;

            double[] qPrev = q[s].clone();
            ik.solve(s, q[s], pT[s], RT[s], 15, 0.2, 0.02, Math.toRadians(180) * dt);
            errPos[s] = ik.errPos;
            errRot[s] = ik.errRot;
            yielded[s] = false;
            if ((ik.errPos > REACH_TOL_M || ik.errRot > REACH_TOL_RAD)
                    && positionPriority && n > 1e-7 && a < 1e-7) {
                // a pure translation the arm cannot do with the orientation held (typically
                // a joint on its limit): retry with the orientation weighted down, keep the
                // position exact and adopt whatever orientation that needs
                // highest orientation weight that still makes the position: least yield
                if (yieldAnchor[s] == null) yieldAnchor[s] = RPrev.clone();
                for (double wr : new double[]{0.1, 0.05, 0.02, 0.01}) {
                    System.arraycopy(qPrev, 0, q[s], 0, 7);
                    ik.solve(s, q[s], pT[s], RT[s], 15, wr, 0.02, Math.toRadians(180) * dt);
                    if (ik.errPos > REACH_TOL_M) continue;
                    double[] pA = new double[3], RA_ = new double[9];
                    ik.fk(s, q[s].clone(), pA, RA_);
                    if (angle(RA_, yieldAnchor[s]) > YIELD_MAX_RAD) break;   // yielded enough: stop
                    RT[s] = RA_;
                    System.arraycopy(RA_, 0, RG[s], 0, 9);
                    errPos[s] = ik.errPos;
                    errRot[s] = 0;
                    yielded[s] = true;
                    break;
                }
            }
            if (!yielded[s] && (ik.errPos > REACH_TOL_M || ik.errRot > REACH_TOL_RAD)) {
                // cannot follow (joint limit / workspace edge): undo this tick - q and the
                // target - and stop there.  Restoring the previous target (not the reached
                // pose) keeps IK residuals from accumulating while a button pushes on a limit,
                // so the arm never drifts in a direction that was not asked for.
                atLimit[s] = true;
                limitHold[s] = 0.3;
                System.arraycopy(qPrev, 0, q[s], 0, 7);
                pT[s] = pPrev;
                RT[s] = RPrev;
                System.arraycopy(pT[s], 0, pG[s], 0, 3);
                System.arraycopy(RT[s], 0, RG[s], 0, 9);
                state[s] = "한계";
            } else {
                atLimit[s] = limitHold[s] > 0;
                state[s] = atLimit[s] ? "한계" : yielded[s] ? "자세 양보" : "이동 중";
            }
        }
    }

    // ------------------------------------------------------------------ internals

    private void tickJoint(int s, double dt) {
        double max = jointSpeed() * dt, worst = 0;
        double[] g = qGoal[s];
        for (int k = 0; k < 7; k++) {
            double d = g[k] - q[s][k];
            worst = Math.max(worst, Math.abs(d));
        }
        double f = worst > max ? max / worst : 1;                   // all joints arrive together
        for (int k = 0; k < 7; k++) q[s][k] += f * (g[k] - q[s][k]);
        ik.fk(s, q[s].clone(), pT[s], RT[s]);
        System.arraycopy(pT[s], 0, pG[s], 0, 3);
        System.arraycopy(RT[s], 0, RG[s], 0, 9);
        atLimit[s] = false;
        state[s] = "관절 이동";
        if (f == 1) qGoal[s] = null;
    }

    /** Delta given in the source arm's jog frame -> goal of that arm (and its mirror). */
    private void applyDelta(int src, double[] v, double[] w) {
        double[] vw = v.clone(), ww = w.clone();
        if (toolFrame) {
            RobotModel.mulv(RG[src], v, vw);
            RobotModel.mulv(RG[src], w, ww);
        }
        for (int s : affected(src)) {
            qGoal[s] = null;
            boolean refl = s != src;                              // reflect about the x-z plane
            double vx = vw[0], vy = refl ? -vw[1] : vw[1], vz = vw[2];
            double wx = refl ? -ww[0] : ww[0], wy = ww[1], wz = refl ? -ww[2] : ww[2];
            pG[s][0] += vx;
            pG[s][1] += vy;
            pG[s][2] += vz;
            double[] Rs = RobotModel.expmap(wx, wy, wz), Rn = new double[9];
            RobotModel.mul(Rs, RG[s], Rn);
            RG[s] = Rn;
        }
    }

    private int[] affected(int side) {
        return mirror ? new int[]{side, 1 - side} : new int[]{side};
    }

    private void resync() {
        for (int s = 0; s < 2; s++) {
            ik.fk(s, q[s].clone(), pT[s], RT[s]);
            System.arraycopy(pT[s], 0, pG[s], 0, 3);
            System.arraycopy(RT[s], 0, RG[s], 0, 9);
            atLimit[s] = false;
            state[s] = "대기";
        }
    }

    private double[] clampQ(int s, double[] v) {
        double[] o = new double[7];
        for (int k = 0; k < 7; k++) o[k] = Math.min(m.qmax[s][k], Math.max(m.qmin[s][k], v[k]));
        return o;
    }

    synchronized void toolPose(int s, double[] p, double[] R) {
        System.arraycopy(pT[s], 0, p, 0, 3);
        System.arraycopy(RT[s], 0, R, 0, 9);
    }

    private static double dist(double[] a, double[] b) {
        double x = a[0] - b[0], y = a[1] - b[1], z = a[2] - b[2];
        return Math.sqrt(x * x + y * y + z * z);
    }

    private static double angle(double[] A, double[] B) {
        double[] Bt = new double[9], d = new double[9];
        RobotModel.transpose(B, Bt);
        RobotModel.mul(A, Bt, d);
        double[] w = RobotModel.rotvec(d);
        return Math.sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    }
}
