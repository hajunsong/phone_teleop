package com.humanoid.teleop;

import android.app.Activity;
import android.content.Context;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.graphics.Typeface;
import android.net.wifi.WifiManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.text.InputType;
import android.view.Gravity;
import android.view.HapticFeedbackConstants;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.TextView;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

/**
 * Phone side of the teleoperation link: two jog pads, no motion sensors.
 *
 * Landscape screen, left half = left arm (LA), right half = right arm (RA), as
 * seen by an operator standing behind the robot facing the same way.  Each
 * button moves that arm's tool target along / about one axis while it is
 * held; releasing stops it.  Several buttons (both arms) can be held at once.
 * The phone sends the button state of both arms at 50 Hz; the PC integrates it at a fixed speed and
 * stops by itself if the stream pauses (watchdog), so a dropped link cannot
 * leave the arm moving.
 *
 * Transport is one TCP connection, the phone being the client:
 *   USB   : PC runs  adb reverse tcp:PORT tcp:PORT,  phone connects 127.0.0.1:PORT
 *   Wi-Fi : phone connects PC_IP:PORT  (PC_IP found by the PC's UDP beacon)
 *
 * Protocol (one line each, UTF-8, '\n'):
 *   phone -> PC  H 4 <model>
 *                K <seq> <speed 0|1|2> <frame B|T> <RA: vx vy vz wx wy wz> <LA: vx vy vz wx wy wz>
 *                  (each axis -1, 0 or +1)
 *                E HOME ALL
 *   PC -> phone  ST <free text>
 */
public class MainActivity extends Activity implements TeleopLink.Listener {

    static final int DEFAULT_PORT = 5001;
    static final int BEACON_PORT = 5000;
    static final int SEND_MS = 20; // 50 Hz

    static final int RA = 0, LA = 1;
    /** axis[arm][k]: k 0..2 translation x y z, 3..5 rotation x y z; -1 / 0 / +1 */
    private final int[][] axis = new int[2][6];
    private long seq;

    private TeleopLink link;
    private final Handler ui = new Handler(Looper.getMainLooper());

    private RadioButton rbUsb, rbWifi, rbSlow, rbMid, rbFast;
    private EditText etHost, etPort;
    private CheckBox cbTool;
    private Button btConnect, btFind, btHome;
    private TextView tvLink, tvPc;

    private final Runnable sender = new Runnable() {
        @Override
        public void run() {
            if (link.isConnected()) {
                link.send(stateLine());
            }
            ui.postDelayed(this, SEND_MS);
        }
    };

    @Override
    protected void onCreate(Bundle b) {
        super.onCreate(b);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        buildUi();
        loadPrefs();
        link = new TeleopLink(this);
    }

    @Override
    protected void onResume() {
        super.onResume();
        ui.post(sender);
    }

    @Override
    protected void onPause() {
        super.onPause();
        java.util.Arrays.fill(axis[RA], 0);
        java.util.Arrays.fill(axis[LA], 0);
        if (link.isConnected()) link.send(stateLine());
        ui.removeCallbacks(sender);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        link.stop();
    }

    private String stateLine() {
        int[] r = axis[RA], l = axis[LA];
        return String.format(Locale.US, "K %d %d %s %d %d %d %d %d %d %d %d %d %d %d %d",
                seq++, speed(), cbTool.isChecked() ? "T" : "B",
                r[0], r[1], r[2], r[3], r[4], r[5], l[0], l[1], l[2], l[3], l[4], l[5]);
    }

    private int speed() {
        return rbSlow.isChecked() ? 0 : rbFast.isChecked() ? 2 : 1;
    }

    // ------------------------------------------------------------------ link callbacks

    @Override
    public void onLinkState(final String text, final boolean connected) {
        ui.post(() -> {
            tvLink.setText(text);
            tvLink.setTextColor(connected ? Color.rgb(0, 140, 60) : Color.rgb(180, 60, 0));
        });
    }

    @Override
    public void onPcLine(final String line) {
        if (line.startsWith("ST ")) ui.post(() -> tvPc.setText(line.substring(3)));
    }

    // ------------------------------------------------------------------ actions

    private void toggleConnect() {
        if (link.isRunning()) {
            link.stop();
            btConnect.setText("연결");
            return;
        }
        savePrefs();
        String host = rbUsb.isChecked() ? "127.0.0.1" : etHost.getText().toString().trim();
        int port;
        try {
            port = Integer.parseInt(etPort.getText().toString().trim());
        } catch (NumberFormatException ex) {
            port = DEFAULT_PORT;
        }
        if (host.isEmpty()) {
            tvLink.setText("PC IP를 입력하거나 'PC 찾기'를 누르세요");
            return;
        }
        link.start(host, port, "H 4 " + Build.MANUFACTURER + "_" + Build.MODEL.replace(' ', '_'));
        btConnect.setText("연결 끊기");
    }

    /** Listen for the PC's UDP beacon  "HUMANOID_TELEOP <port> <host>"  for 3 s. */
    private void findPc() {
        btFind.setEnabled(false);
        tvLink.setText("PC 찾는 중 (UDP " + BEACON_PORT + ")...");
        new Thread(() -> {
            WifiManager wm = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE);
            WifiManager.MulticastLock lock = wm.createMulticastLock("teleop-beacon");
            lock.acquire();
            String found = null;
            int foundPort = DEFAULT_PORT;
            try (DatagramSocket s = new DatagramSocket(null)) {
                s.setReuseAddress(true);
                s.setBroadcast(true);
                s.bind(new InetSocketAddress(BEACON_PORT));
                s.setSoTimeout(3000);
                byte[] buf = new byte[256];
                DatagramPacket p = new DatagramPacket(buf, buf.length);
                s.receive(p);
                String msg = new String(p.getData(), 0, p.getLength(), StandardCharsets.UTF_8).trim();
                String[] t = msg.split("\\s+");
                if (t.length >= 2 && t[0].equals("HUMANOID_TELEOP")) {
                    found = p.getAddress().getHostAddress();
                    foundPort = Integer.parseInt(t[1]);
                }
            } catch (SocketTimeoutException ex) {
                // nothing heard
            } catch (Exception ex) {
                final String err = ex.toString();
                ui.post(() -> tvLink.setText("찾기 실패: " + err));
            } finally {
                lock.release();
            }
            final String host = found;
            final int port = foundPort;
            ui.post(() -> {
                btFind.setEnabled(true);
                if (host != null) {
                    etHost.setText(host);
                    etPort.setText(String.valueOf(port));
                    tvLink.setText("PC 발견: " + host + ":" + port);
                } else if (tvLink.getText().toString().startsWith("PC 찾는")) {
                    tvLink.setText("PC 비컨 없음 — PC 프로그램 실행/같은 Wi-Fi/방화벽 확인");
                }
            });
        }, "beacon").start();
    }

    // ------------------------------------------------------------------ UI

    private void buildUi() {
        final int pad = dp(10);
        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setPadding(pad, pad, pad, pad);
        root.setBackgroundColor(Color.rgb(245, 245, 245));
        // Android 15+ draws edge-to-edge for targetSdk 35: keep clear of the system bars
        root.setOnApplyWindowInsetsListener((v, ins) -> {
            int l, t, r, bt;
            if (Build.VERSION.SDK_INT >= 30) {
                android.graphics.Insets i = ins.getInsets(
                        android.view.WindowInsets.Type.systemBars() | android.view.WindowInsets.Type.displayCutout());
                l = i.left; t = i.top; r = i.right; bt = i.bottom;
            } else {
                l = ins.getSystemWindowInsetLeft(); t = ins.getSystemWindowInsetTop();
                r = ins.getSystemWindowInsetRight(); bt = ins.getSystemWindowInsetBottom();
            }
            v.setPadding(pad + l, pad + t, pad + r, pad + bt);
            return ins;
        });

        // row 1: transport, address, connect, home
        LinearLayout top = row();
        RadioGroup rgMode = new RadioGroup(this);
        rgMode.setOrientation(RadioGroup.HORIZONTAL);
        rbUsb = radio("USB");
        rbWifi = radio("Wi-Fi");
        rgMode.addView(rbUsb);
        rgMode.addView(rbWifi);
        top.addView(rgMode);
        etHost = new EditText(this);
        etHost.setHint("PC IP (Wi-Fi)");
        etHost.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_URI);
        etHost.setSingleLine(true);
        top.addView(etHost, new LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 2.2f));
        etPort = new EditText(this);
        etPort.setHint("port");
        etPort.setInputType(InputType.TYPE_CLASS_NUMBER);
        etPort.setSingleLine(true);
        top.addView(etPort, new LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 0.9f));
        btFind = new Button(this);
        btFind.setText("PC 찾기");
        btFind.setOnClickListener(v -> findPc());
        top.addView(btFind);
        btConnect = new Button(this);
        btConnect.setText("연결");
        btConnect.setOnClickListener(v -> toggleConnect());
        top.addView(btConnect, new LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1.3f));
        btHome = new Button(this);
        btHome.setText("양팔 초기자세");
        btHome.setOnClickListener(v -> link.send("E HOME ALL"));
        top.addView(btHome, new LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1.6f));
        root.addView(top);

        rgMode.setOnCheckedChangeListener((g, id) -> {
            boolean wifi = rbWifi.isChecked();
            etHost.setEnabled(wifi);
            btFind.setEnabled(wifi);
        });

        // row 2: speed, frame, link + PC status
        LinearLayout opt = row();
        TextView spLabel = new TextView(this);
        spLabel.setText("속도:");
        opt.addView(spLabel);
        RadioGroup rgSpeed = new RadioGroup(this);
        rgSpeed.setOrientation(RadioGroup.HORIZONTAL);
        rbSlow = radio("느림");
        rbMid = radio("보통");
        rbFast = radio("빠름");
        rgSpeed.addView(rbSlow);
        rgSpeed.addView(rbMid);
        rgSpeed.addView(rbFast);
        opt.addView(rgSpeed);
        cbTool = new CheckBox(this);
        cbTool.setText("툴 좌표계");
        opt.addView(cbTool);
        tvLink = text(13);
        tvLink.setText("연결 안 됨");
        tvLink.setPadding(dp(12), 0, dp(12), 0);
        opt.addView(tvLink);
        tvPc = text(13);
        tvPc.setText("PC 상태: -");
        tvPc.setTextColor(Color.rgb(20, 60, 160));
        tvPc.setSingleLine(true);
        opt.addView(tvPc, new LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f));
        root.addView(opt);

        // pads: left half LA, right half RA
        LinearLayout pads = row();
        pads.setGravity(Gravity.FILL);
        pads.setMotionEventSplittingEnabled(true);
        LinearLayout.LayoutParams plp = new LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.MATCH_PARENT, 1f);
        plp.rightMargin = dp(8);
        pads.addView(armPad(LA, "왼팔 LA"), plp);
        LinearLayout.LayoutParams prp = new LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.MATCH_PARENT, 1f);
        prp.leftMargin = dp(8);
        pads.addView(armPad(RA, "오른팔 RA"), prp);
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1f);
        lp.topMargin = dp(4);
        root.addView(pads, lp);
        root.setMotionEventSplittingEnabled(true);

        setContentView(root);
    }

    /** One arm: header + 3 rows of [T-][T+][R-][R+]  (X/Rx, Y/Ry, Z/Rz). */
    private LinearLayout armPad(final int arm, String name) {
        LinearLayout col = new LinearLayout(this);
        col.setOrientation(LinearLayout.VERTICAL);
        col.setMotionEventSplittingEnabled(true);
        TextView h = new TextView(this);
        h.setText(name);
        h.setTextSize(15);
        h.setTypeface(Typeface.DEFAULT_BOLD);
        h.setTextColor(Color.BLACK);
        h.setGravity(Gravity.CENTER);
        col.addView(h);
        String[][] t = {{"X− 뒤", "X+ 앞"}, {"Y− 오른", "Y+ 왼"}, {"Z− 아래", "Z+ 위"}};
        String[][] r = {{"Rx−", "Rx+"}, {"Ry−", "Ry+"}, {"Rz−", "Rz+"}};
        int cT = Color.rgb(40, 90, 160), cR = Color.rgb(150, 80, 30);
        for (int k = 0; k < 3; k++) {
            LinearLayout rw = row();
            rw.setMotionEventSplittingEnabled(true);
            rw.addView(jogButton(t[k][0], arm, k, -1, cT), jogLp());
            rw.addView(jogButton(t[k][1], arm, k, +1, cT), jogLp());
            LinearLayout.LayoutParams gap = jogLp();
            gap.leftMargin = dp(8);
            rw.addView(jogButton(r[k][0], arm, k + 3, -1, cR), gap);
            rw.addView(jogButton(r[k][1], arm, k + 3, +1, cR), jogLp());
            LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(
                    ViewGroup.LayoutParams.MATCH_PARENT, 0, 1f);
            lp.topMargin = dp(3);
            col.addView(rw, lp);
        }
        return col;
    }

    private LinearLayout.LayoutParams jogLp() {
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.MATCH_PARENT, 1f);
        lp.leftMargin = dp(3);
        lp.rightMargin = dp(3);
        return lp;
    }

    /** Hold-to-move button for one direction of one axis. */
    private Button jogButton(String label, final int arm, final int k, final int dir, final int color) {
        final Button b = new Button(this);
        b.setText(label);
        b.setTextSize(16);
        b.setTextColor(Color.WHITE);
        b.setAllCaps(false);
        b.setBackgroundColor(color);
        b.setOnTouchListener((v, ev) -> {
            int a = ev.getActionMasked();
            if (a == MotionEvent.ACTION_DOWN) {
                axis[arm][k] = dir;
                b.setBackgroundColor(Color.rgb(0, 160, 70));
                b.performHapticFeedback(HapticFeedbackConstants.VIRTUAL_KEY);
            } else if (a == MotionEvent.ACTION_UP || a == MotionEvent.ACTION_CANCEL) {
                if (axis[arm][k] == dir) axis[arm][k] = 0;
                b.setBackgroundColor(color);
            }
            return true;
        });
        return b;
    }

    private RadioButton radio(String s) {
        RadioButton r = new RadioButton(this);
        r.setText(s);
        r.setId(View.generateViewId());
        return r;
    }

    private LinearLayout row() {
        LinearLayout l = new LinearLayout(this);
        l.setOrientation(LinearLayout.HORIZONTAL);
        l.setGravity(Gravity.CENTER_VERTICAL);
        return l;
    }

    private TextView text(int sp) {
        TextView t = new TextView(this);
        t.setTextSize(sp);
        t.setPadding(0, dp(3), 0, 0);
        return t;
    }

    private int dp(int v) {
        return Math.round(v * getResources().getDisplayMetrics().density);
    }

    private void loadPrefs() {
        SharedPreferences p = getSharedPreferences("teleop", MODE_PRIVATE);
        boolean wifi = p.getBoolean("wifi", false);
        rbUsb.setChecked(!wifi);
        rbWifi.setChecked(wifi);
        etHost.setEnabled(wifi);
        btFind.setEnabled(wifi);
        etHost.setText(p.getString("host", ""));
        etPort.setText(String.valueOf(p.getInt("port", DEFAULT_PORT)));
        int sp = p.getInt("speed", 1);
        rbSlow.setChecked(sp == 0);
        rbMid.setChecked(sp == 1);
        rbFast.setChecked(sp == 2);
        cbTool.setChecked(p.getBoolean("toolFrame", false));
    }

    private void savePrefs() {
        int port;
        try {
            port = Integer.parseInt(etPort.getText().toString().trim());
        } catch (NumberFormatException ex) {
            port = DEFAULT_PORT;
        }
        getSharedPreferences("teleop", MODE_PRIVATE).edit()
                .putBoolean("wifi", rbWifi.isChecked())
                .putString("host", etHost.getText().toString().trim())
                .putInt("port", port)
                .putInt("speed", speed())
                .putBoolean("toolFrame", cbTool.isChecked())
                .apply();
    }
}
