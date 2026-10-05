import com.humanoid.teleop.RobotModel;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.InputStreamReader;
import java.util.Locale;

/**
 * Desktop check of the phone kinematics (run by tools/check_phone_kinematics.py).
 *   stdin  "F q_RA(7) q_LA(7)"                 -> "F" + 3 pos per body + RA tool p,R + LA tool p,R
 *   stdin  "I side q0(7) p(3) R(9)"            -> "I" + q(7) errPos errRot
 */
public class KinCheck {
    public static void main(String[] a) throws Exception {
        RobotModel m = RobotModel.load(new FileInputStream(a[0]));
        RobotModel.Ik ik = m.new Ik();
        int n = m.bodies.size();
        double[][] pos = new double[n][], rot = new double[n][];
        BufferedReader in = new BufferedReader(new InputStreamReader(System.in));
        StringBuilder sb = new StringBuilder();
        String line;
        while ((line = in.readLine()) != null) {
            String[] t = line.trim().split("\\s+");
            sb.setLength(0);
            if (t[0].equals("F")) {
                double[][] q = new double[2][7];
                for (int i = 0; i < 14; i++) q[i / 7][i % 7] = Double.parseDouble(t[1 + i]);
                m.fkAll(q, pos, rot);
                sb.append("F");
                for (int b = 1; b < n; b++) for (int i = 0; i < 3; i++) sb.append(' ').append(pos[b][i]);
                double[] p = new double[3], R = new double[9];
                for (int s = 0; s < 2; s++) {
                    m.tool(s, pos, rot, p, R);
                    for (double v : p) sb.append(' ').append(v);
                    for (double v : R) sb.append(' ').append(v);
                }
            } else {
                int s = t[1].equals("LA") ? 1 : 0;
                double[] q = new double[7], p = new double[3], R = new double[9];
                for (int i = 0; i < 7; i++) q[i] = Double.parseDouble(t[2 + i]);
                for (int i = 0; i < 3; i++) p[i] = Double.parseDouble(t[9 + i]);
                for (int i = 0; i < 9; i++) R[i] = Double.parseDouble(t[12 + i]);
                long t0 = System.nanoTime();
                ik.solve(s, q, p, R, 200, 0.2, 0.02, 0.2);
                double ms = (System.nanoTime() - t0) / 1e6;
                sb.append("I");
                for (double v : q) sb.append(' ').append(v);
                sb.append(' ').append(ik.errPos).append(' ').append(ik.errRot).append(' ').append(String.format(Locale.US, "%.3f", ms));
            }
            System.out.println(sb);
        }
    }
}
