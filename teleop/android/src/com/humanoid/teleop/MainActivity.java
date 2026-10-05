package com.humanoid.teleop;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.net.wifi.WifiManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.text.InputType;
import android.view.Gravity;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowInsets;
import android.view.WindowInsetsController;
import android.view.WindowManager;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.ScrollView;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import org.json.JSONArray;
import org.json.JSONObject;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.Locale;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Upper-body jog teleoperation: the phone solves the IK and publishes joint
 * targets to the robot's MQTT topic itself.
 *
 *   publish   /humanoid/upper/cmd/joint    [0, 0, LA q1..q7, RA q1..q7, servo(, gain)]   (rad, 50 Hz)
 *   subscribe /humanoid/upper/joint_state  {"pos_rad": [16]}  -> start from the robot's actual pose
 *
 * USB: the PC runs the broker and  adb reverse tcp:1883 tcp:1883, the phone connects to 127.0.0.1.
 * Wi-Fi: the phone connects to the broker's IP (PC simulation, or the robot's broker).
 *
 * Safety: nothing is published until the robot's joint_state has been received
 * and adopted (unless explicitly allowed in settings); every motion is rate
 * limited and continuous; joint limits are the robot's measured travel.
 */
public class MainActivity extends Activity {

    static final int BEACON_PORT = 5000;
    static final int RA = RobotModel.RA, LA = RobotModel.LA;

    // palette
    static final int BLUE = Color.rgb(29, 95, 216), BLUE_L = Color.rgb(222, 233, 252), BLUE_T = Color.rgb(30, 80, 190);
    static final int BROWN = Color.rgb(150, 82, 30), BROWN_L = Color.rgb(251, 228, 210), BROWN_T = Color.rgb(170, 80, 25);
    static final int BG = Color.rgb(241, 243, 247), CARD = Color.WHITE, INK = Color.rgb(28, 32, 40), MUTED = Color.rgb(110, 116, 128);
    static final int GREEN = Color.rgb(40, 160, 90), GREEN_L = Color.rgb(218, 244, 226), AMBER = Color.rgb(205, 140, 20),
            RED = Color.rgb(205, 55, 55), GREY_L = Color.rgb(228, 231, 236);

    private RobotModel model;
    private Controller ctl;
    private RobotView robotView;
    private final Handler ui = new Handler(Looper.getMainLooper());
    private ScheduledExecutorService loop;

    // MQTT
    private volatile MqttLite mqtt;
    private volatile boolean connecting, published, syncPending;
    private volatile long syncStart;
    private volatile double[][] robotQ;            // last joint_state
    private volatile long robotQTime;
    private volatile String host = "";
    private volatile int port = 1883;
    private volatile String linkNote = "";
    private long nPub;

    // settings
    private String topicCmd, topicState, clientId;
    private boolean servoFlag, allowWithoutState;
    private String gain;
    private int pubHz;

    // views
    private RadioButton rbUsb, rbWifi, rbSlow, rbMid, rbFast;
    private EditText etHost, etPort;
    private TextView btFind, chipTitle, chipSub, btConnect;
    private LinearLayout chip;
    private final TextView[] badge = new TextView[2];
    private final int[] mmIdx = {2, 2}, degIdx = {2, 2};            // index into MM / DEG
    private final TextView[] stepLabelMm = new TextView[2], stepLabelDeg = new TextView[2];
    private SeekBar sbSpeed;
    private CheckBox cbContinuous, cbFrames, cbMirror, cbTool;
    private boolean syncingSpeed;
    /** "RA:2:1" -> jog key view (side : axis 0..5 : dir), for the multi-touch self test. */
    private final java.util.Map<String, View> jogKeys = new java.util.HashMap<>();

    static final String[] MM = {"1", "2", "5", "10", "20", "50"};
    static final String[] DEG = {"0.5", "1", "2", "5", "10", "20"};

    // ------------------------------------------------------------------ lifecycle

    @Override
    protected void onCreate(Bundle b) {
        super.onCreate(b);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        try {
            model = RobotModel.load(getAssets().open("robot_model.txt"));
        } catch (Exception e) {
            throw new RuntimeException("robot_model.txt", e);
        }
        ctl = new Controller(model);
        loadSettings();
        buildUi();
        loadPrefs();
        hideSystemBars();
    }

    /** Key visuals to restore when every finger has left the screen. */
    private final java.util.List<Runnable> keyResets = new java.util.ArrayList<>();

    /**
     * Safety net for simultaneous presses: when the last finger leaves the screen
     * (or the system cancels the gesture), every jog is released, even if some
     * key never saw its own UP.
     */
    @Override
    public boolean dispatchTouchEvent(MotionEvent ev) {
        boolean r = super.dispatchTouchEvent(ev);
        int a = ev.getActionMasked();
        if (a == MotionEvent.ACTION_UP || a == MotionEvent.ACTION_CANCEL) {
            ctl.releaseAll();
            for (Runnable k : keyResets) k.run();
        }
        return r;
    }

    @Override
    protected void onNewIntent(android.content.Intent i) {
        super.onNewIntent(i);
        setIntent(i);
        ui.postDelayed(this::runMultiTouchTest, 500);
    }

    /**
     * Multi-touch self test (for development, triggered from adb):
     *   adb shell am start -n com.humanoid.teleop/.MainActivity --es mt "LA:2:1,RA:0:1" --ei mt_ms 1500
     * presses all listed jog keys at once with real multi-pointer MotionEvents sent
     * to the window's root view, i.e. through the same dispatch/splitting path a
     * hand takes, holds them, then lifts them one by one.
     */
    private void runMultiTouchTest() {
        android.content.Intent it = getIntent();
        String spec = it == null ? null : it.getStringExtra("mt");
        if (spec == null) return;
        it.removeExtra("mt");
        final int ms = it.getIntExtra("mt_ms", 1500);
        final String[] keys = spec.split(",");
        final View root = getWindow().getDecorView();
        final float[][] pts = new float[keys.length][];
        for (int i = 0; i < keys.length; i++) {
            View v = jogKeys.get(keys[i].trim());
            if (v == null) {
                toast("mt: no key " + keys[i]);
                return;
            }
            int[] xy = new int[2];
            v.getLocationInWindow(xy);
            pts[i] = new float[]{xy[0] + v.getWidth() / 2f, xy[1] + v.getHeight() / 2f};
        }
        final long t0 = android.os.SystemClock.uptimeMillis();
        for (int n = 1; n <= keys.length; n++) {                  // fingers go down one after another
            final int cnt = n;
            ui.postDelayed(() -> root.dispatchTouchEvent(mtEvent(t0, cnt, cnt == 1
                    ? MotionEvent.ACTION_DOWN : MotionEvent.ACTION_POINTER_DOWN, cnt - 1, pts)), 30L * n);
        }
        for (int n = keys.length; n >= 1; n--) {                  // and lift in reverse order
            final int cnt = n;
            ui.postDelayed(() -> root.dispatchTouchEvent(mtEvent(t0, cnt, cnt == 1
                    ? MotionEvent.ACTION_UP : MotionEvent.ACTION_POINTER_UP, cnt - 1, pts)),
                    30L * keys.length + ms + 30L * (keys.length - n));
        }
    }

    private static MotionEvent mtEvent(long down, int count, int action, int index, float[][] pts) {
        MotionEvent.PointerProperties[] pp = new MotionEvent.PointerProperties[count];
        MotionEvent.PointerCoords[] pc = new MotionEvent.PointerCoords[count];
        for (int i = 0; i < count; i++) {
            pp[i] = new MotionEvent.PointerProperties();
            pp[i].id = i;
            pp[i].toolType = MotionEvent.TOOL_TYPE_FINGER;
            pc[i] = new MotionEvent.PointerCoords();
            pc[i].x = pts[i][0];
            pc[i].y = pts[i][1];
            pc[i].pressure = 1;
            pc[i].size = 1;
        }
        int act = action | (index << MotionEvent.ACTION_POINTER_INDEX_SHIFT);
        return MotionEvent.obtain(down, android.os.SystemClock.uptimeMillis(), act, count, pp, pc,
                0, 0, 1, 1, 0, 0, android.view.InputDevice.SOURCE_TOUCHSCREEN, 0);
    }

    @Override
    protected void onResume() {
        super.onResume();
        robotView.onResume();
        hideSystemBars();
        loop = Executors.newSingleThreadScheduledExecutor();
        final long[] k = {0};
        loop.scheduleAtFixedRate(() -> {
            try {
                controlTick(k[0]++);
            } catch (Throwable t) {
                ui.post(() -> Toast.makeText(this, "control: " + t, Toast.LENGTH_LONG).show());
            }
        }, 0, 10, TimeUnit.MILLISECONDS);
        ui.post(statusTask);
        if (mqtt != null && mqtt.isOpen()) beginSync();      // came back: adopt the robot pose again
        ui.postDelayed(this::runMultiTouchTest, 2500);         // after a re-sync, if requested
    }

    @Override
    protected void onPause() {
        super.onPause();
        ctl.releaseAll();
        robotView.onPause();
        if (loop != null) loop.shutdownNow();
        ui.removeCallbacks(statusTask);
        savePrefs();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        disconnect();
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        if (hasFocus) hideSystemBars();
    }

    private void hideSystemBars() {
        if (Build.VERSION.SDK_INT >= 30) {
            WindowInsetsController c = getWindow().getInsetsController();
            if (c != null) {
                c.hide(WindowInsets.Type.systemBars());
                c.setSystemBarsBehavior(WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE);
            }
        } else {
            getWindow().getDecorView().setSystemUiVisibility(View.SYSTEM_UI_FLAG_FULLSCREEN
                    | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
        }
    }

    // ------------------------------------------------------------------ control loop (100 Hz)

    private void controlTick(long k) {
        MqttLite m = mqtt;
        boolean open = m != null && m.isOpen();
        if (open && syncPending) {
            double[][] rq = robotQ;
            if (rq != null && robotQTime >= syncStart) {
                ctl.syncFrom(rq);
                syncPending = false;
                published = true;
                linkNote = "실기/시뮬 자세로 동기화";
            } else if (System.currentTimeMillis() - syncStart > 1500) {
                if (allowWithoutState) {
                    syncPending = false;
                    published = true;
                    linkNote = "상태 미수신 — 폰 자세로 시작";
                } else {
                    linkNote = "상태 미수신 — 발행 보류 (설정에서 허용 가능)";
                }
            }
        }
        if (!syncPending) ctl.tick(0.01);
        int every = Math.max(1, Math.round(100f / Math.max(1, pubHz)));
        if (open && published && k % every == 0) {
            double[][] q = ctl.snapshotQ();
            StringBuilder sb = new StringBuilder(260).append("[0,0");
            for (int s : new int[]{LA, RA})
                for (int i = 0; i < 7; i++) sb.append(',').append(String.format(Locale.US, "%.6f", q[s][i]));
            sb.append(',').append(servoFlag ? 1 : 0);
            if (gain != null && !gain.isEmpty()) sb.append(',').append(gain);
            sb.append(']');
            try {
                m.publish(topicCmd, sb.toString().getBytes(StandardCharsets.UTF_8));
                nPub++;
            } catch (Exception e) {
                m.close();
            }
        }
    }

    // ------------------------------------------------------------------ MQTT

    private void toggleConnect() {
        if (mqtt != null || connecting) {
            disconnect();
            return;
        }
        savePrefs();
        host = rbUsb.isChecked() ? "127.0.0.1" : etHost.getText().toString().trim();
        try {
            port = Integer.parseInt(etPort.getText().toString().trim());
        } catch (NumberFormatException e) {
            port = 1883;
        }
        if (host.isEmpty()) {
            toast("브로커 IP를 입력하거나 'PC 찾기'를 누르세요");
            return;
        }
        connecting = true;
        linkNote = "";
        new Thread(() -> {
            MqttLite m = new MqttLite(20);
            try {
                m.connect(host, port, clientId, new MqttLite.Listener() {
                    @Override
                    public void onMessage(String topic, byte[] payload) {
                        if (topic.equals(topicState)) onState(payload);
                    }

                    @Override
                    public void onClosed(String reason) {
                        linkNote = "끊김: " + reason;
                        mqtt = null;
                        published = false;
                    }
                });
                m.subscribe(topicState);
                mqtt = m;
                beginSync();
            } catch (Exception e) {
                linkNote = "연결 실패: " + (e.getMessage() == null ? e.toString() : e.getMessage());
            } finally {
                connecting = false;
            }
        }, "mqtt-connect").start();
    }

    private void beginSync() {
        published = false;
        syncStart = System.currentTimeMillis();
        syncPending = true;
        linkNote = "로봇 자세 수신 대기…";
    }

    private void disconnect() {
        MqttLite m = mqtt;
        mqtt = null;
        published = false;
        syncPending = false;
        if (m != null) new Thread(m::close).start();
        linkNote = "";
    }

    private void onState(byte[] payload) {
        try {
            JSONArray a = new JSONObject(new String(payload, StandardCharsets.UTF_8)).getJSONArray("pos_rad");
            if (a.length() < 16) return;
            double[][] q = new double[2][7];
            for (int i = 0; i < 7; i++) {
                q[LA][i] = a.getDouble(2 + i);
                q[RA][i] = a.getDouble(9 + i);
            }
            robotQ = q;
            robotQTime = System.currentTimeMillis();
        } catch (Exception ignored) {
        }
    }

    /** Listen for the PC's UDP beacon  "HUMANOID_TELEOP <port> <host>"  for 3 s. */
    private void findPc() {
        btFind.setEnabled(false);
        btFind.setText("찾는 중…");
        new Thread(() -> {
            WifiManager wm = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE);
            WifiManager.MulticastLock lock = wm.createMulticastLock("teleop-beacon");
            lock.acquire();
            String found = null;
            int fport = 1883;
            try (DatagramSocket s = new DatagramSocket(null)) {
                s.setReuseAddress(true);
                s.setBroadcast(true);
                s.bind(new InetSocketAddress(BEACON_PORT));
                s.setSoTimeout(3000);
                byte[] buf = new byte[256];
                DatagramPacket p = new DatagramPacket(buf, buf.length);
                s.receive(p);
                String[] t = new String(p.getData(), 0, p.getLength(), StandardCharsets.UTF_8).trim().split("\\s+");
                if (t.length >= 2 && t[0].equals("HUMANOID_TELEOP")) {
                    found = p.getAddress().getHostAddress();
                    fport = Integer.parseInt(t[1]);
                }
            } catch (SocketTimeoutException ignored) {
            } catch (Exception e) {
                linkNote = "찾기 실패: " + e;
            } finally {
                lock.release();
            }
            final String h = found;
            final int pp = fport;
            ui.post(() -> {
                btFind.setEnabled(true);
                btFind.setText("⌕  PC 찾기");
                if (h != null) {
                    rbWifi.setChecked(true);
                    etHost.setText(h);
                    etPort.setText(String.valueOf(pp));
                    toast("PC 발견: " + h + ":" + pp);
                } else {
                    toast("비컨 없음 — PC 프로그램 실행 / 같은 Wi-Fi / 방화벽 확인");
                }
            });
        }, "beacon").start();
    }

    // ------------------------------------------------------------------ status (10 Hz)

    private final Runnable statusTask = new Runnable() {
        @Override
        public void run() {
            MqttLite m = mqtt;
            boolean open = m != null && m.isOpen();
            if (open && published) {
                setChip("연결됨", host + ":" + port + (linkNote.isEmpty() ? "" : "  ·  " + linkNote), GREEN, GREEN_L);
                btConnect.setText("연결 끊기");
            } else if (open || connecting) {
                setChip(connecting ? "연결 중…" : "동기화 중", linkNote.isEmpty() ? host + ":" + port : linkNote,
                        AMBER, Color.rgb(252, 241, 214));
                btConnect.setText("연결 끊기");
            } else {
                setChip("연결 안 됨", linkNote.isEmpty() ? "브로커에 연결하세요" : linkNote, MUTED, GREY_L);
                btConnect.setText("연결");
            }
            for (int s = 0; s < 2; s++) {
                String st = ctl.state[s];
                int c;
                String txt;
                if (!(open && published)) {
                    c = MUTED;
                    txt = "대기";
                } else if (st.equals("한계")) {
                    c = RED;
                    txt = "한계";
                } else if (st.equals("자세 양보")) {
                    c = AMBER;
                    txt = "자세 양보";
                } else if (st.equals("대기")) {
                    c = GREEN;
                    txt = "활성";
                } else {
                    c = GREEN;
                    txt = st;
                }
                badge[s].setText("●  " + txt);
                badge[s].setTextColor(c == GREEN ? Color.rgb(170, 245, 190) : c == MUTED ? Color.rgb(225, 228, 235)
                        : c == RED ? Color.rgb(255, 200, 200) : Color.rgb(255, 228, 170));
            }
            ui.postDelayed(this, 100);
        }
    };

    private void setChip(String title, String sub, int fg, int bg) {
        chipTitle.setText(title);
        chipTitle.setTextColor(fg == MUTED ? INK : fg);
        chipSub.setText(sub);
        chip.setBackground(round(bg, 12));
    }

    // ------------------------------------------------------------------ UI

    private void buildUi() {
        LinearLayout root = vbox();
        root.setBackgroundColor(BG);
        root.setPadding(dp(8), dp(6), dp(8), dp(6));
        root.addView(topBar(), new LinearLayout.LayoutParams(-1, dp(54)));

        LinearLayout mid = hbox();
        mid.setMotionEventSplittingEnabled(true);
        LinearLayout.LayoutParams lpL = new LinearLayout.LayoutParams(0, -1, 1f);
        lpL.rightMargin = dp(6);
        mid.addView(armPanel(LA), lpL);
        LinearLayout.LayoutParams lpC = new LinearLayout.LayoutParams(0, -1, 0.74f);
        lpC.rightMargin = dp(6);
        mid.addView(centerPanel(), lpC);
        mid.addView(armPanel(RA), new LinearLayout.LayoutParams(0, -1, 1f));
        LinearLayout.LayoutParams lpm = new LinearLayout.LayoutParams(-1, 0, 1f);
        lpm.topMargin = dp(6);
        lpm.bottomMargin = dp(6);
        root.addView(mid, lpm);
        root.addView(bottomBar(), new LinearLayout.LayoutParams(-1, dp(46)));
        root.setMotionEventSplittingEnabled(true);
        setContentView(root);
    }

    private View topBar() {
        LinearLayout bar = hbox();
        bar.setMotionEventSplittingEnabled(true);
        bar.setBackground(round(CARD, 14));
        bar.setPadding(dp(10), dp(2), dp(8), dp(2));
        bar.setGravity(Gravity.CENTER_VERTICAL);

        LinearLayout c1 = labeled("연결 방식");
        RadioGroup rg = new RadioGroup(this);
        rg.setOrientation(RadioGroup.HORIZONTAL);
        rbUsb = radio("USB");
        rbWifi = radio("Wi-Fi");
        rg.addView(rbUsb);
        rg.addView(rbWifi);
        rg.setOnCheckedChangeListener((g, id) -> etHost.setEnabled(rbWifi.isChecked()));
        c1.addView(rg);
        bar.addView(c1);
        bar.addView(divider());

        LinearLayout c2 = labeled("IP 주소 (브로커)");
        etHost = input("192.168.45.35", InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_URI);
        c2.addView(etHost, new LinearLayout.LayoutParams(-1, dp(30)));
        bar.addView(c2, new LinearLayout.LayoutParams(0, -2, 1.5f));
        LinearLayout c3 = labeled("포트");
        etPort = input("1883", InputType.TYPE_CLASS_NUMBER);
        c3.addView(etPort, new LinearLayout.LayoutParams(-1, dp(30)));
        LinearLayout.LayoutParams lp3 = new LinearLayout.LayoutParams(0, -2, 0.55f);
        lp3.leftMargin = dp(6);
        bar.addView(c3, lp3);

        btFind = button("⌕  PC 찾기", BLUE, Color.WHITE, 13);
        btFind.setOnClickListener(v -> findPc());
        bar.addView(btFind, margins(new LinearLayout.LayoutParams(0, dp(38), 0.85f), 8, 0));

        chip = vbox();
        chip.setGravity(Gravity.CENTER_VERTICAL);
        chip.setPadding(dp(10), 0, dp(6), 0);
        chipTitle = text("연결 안 됨", 14, INK, true);
        chipSub = text("", 10, MUTED, false);
        chipSub.setSingleLine(true);
        chip.addView(chipTitle);
        chip.addView(chipSub);
        bar.addView(chip, margins(new LinearLayout.LayoutParams(0, dp(42), 1.5f), 8, 0));

        btConnect = button("연결", GREY_L, INK, 13);
        btConnect.setOnClickListener(v -> toggleConnect());
        bar.addView(btConnect, margins(new LinearLayout.LayoutParams(0, dp(38), 0.75f), 6, 0));
        TextView set = button("⚙  설정", GREY_L, INK, 13);
        set.setOnClickListener(v -> showSettings());
        bar.addView(set, margins(new LinearLayout.LayoutParams(0, dp(38), 0.75f), 6, 0));
        TextView home = button("⌂  양팔 초기자세", GREY_L, INK, 13);
        home.setOnClickListener(v -> ctl.moveHome(-1));
        bar.addView(home, margins(new LinearLayout.LayoutParams(0, dp(38), 1.15f), 6, 0));
        return bar;
    }

    private View armPanel(final int side) {
        boolean la = side == LA;
        int main = la ? BLUE : BROWN;
        LinearLayout p = vbox();
        p.setBackground(round(CARD, 14));
        p.setMotionEventSplittingEnabled(true);

        LinearLayout head = hbox();
        head.setGravity(Gravity.CENTER_VERTICAL);
        head.setPadding(dp(12), 0, dp(8), 0);
        GradientDrawable hd = new GradientDrawable();
        hd.setColor(main);
        float r = dp(14);
        hd.setCornerRadii(new float[]{r, r, r, r, 0, 0, 0, 0});
        head.setBackground(hd);
        head.addView(text(la ? "◖ 왼팔 (LA)" : "◖ 오른팔 (RA)", 15, Color.WHITE, true),
                new LinearLayout.LayoutParams(0, -2, 1f));
        badge[side] = text("●  대기", 12, Color.WHITE, true);
        badge[side].setBackground(round(Color.argb(60, 0, 0, 0), 12));
        badge[side].setPadding(dp(10), dp(2), dp(10), dp(2));
        head.addView(badge[side]);
        p.addView(head, new LinearLayout.LayoutParams(-1, dp(32)));

        LinearLayout body = hbox();
        body.setPadding(dp(6), dp(6), dp(6), dp(6));
        body.setMotionEventSplittingEnabled(true);
        body.addView(padCard(side, false), margins(new LinearLayout.LayoutParams(0, -1, 1f), 0, 3));
        body.addView(padCard(side, true), margins(new LinearLayout.LayoutParams(0, -1, 1f), 3, 0));
        p.addView(body, new LinearLayout.LayoutParams(-1, 0, 1f));
        return p;
    }

    /** One 4x3 pad: position (X/Y/Z) or orientation (Rx/Ry/Rz), with its step selector. */
    private View padCard(final int side, final boolean rot) {
        LinearLayout c = vbox();
        c.setBackground(stroke(CARD, Color.rgb(230, 233, 239), 10));
        c.setPadding(dp(5), dp(4), dp(5), dp(5));
        c.setMotionEventSplittingEnabled(true);

        LinearLayout title = hbox();
        title.setGravity(Gravity.CENTER_VERTICAL);
        TextView tt = text(rot ? "자세 (R)" : "위치 (XYZ)", 12, INK, true);
        tt.setSingleLine(true);
        title.addView(tt, new LinearLayout.LayoutParams(0, -2, 1f));
        final String[] opts = rot ? DEG : MM;
        final int[] idx = rot ? degIdx : mmIdx;
        final String unit = rot ? "°" : " mm";
        final TextView sp = button(opts[idx[side]] + unit + " ▾", CARD, INK, 12);
        sp.setBackground(stroke(CARD, Color.rgb(205, 210, 220), 6));
        sp.setOnClickListener(v -> {
            android.widget.PopupMenu pm = new android.widget.PopupMenu(this, sp);
            for (int i = 0; i < opts.length; i++) pm.getMenu().add(0, i, i, opts[i] + (rot ? " °" : " mm"));
            pm.setOnMenuItemClickListener(it -> {
                idx[side] = it.getItemId();
                sp.setText(opts[idx[side]] + unit + " ▾");
                return true;
            });
            pm.setOnDismissListener(d -> hideSystemBars());
            pm.show();
        });
        title.addView(sp, new LinearLayout.LayoutParams(dp(62), dp(24)));
        if (rot) stepLabelDeg[side] = sp;
        else stepLabelMm[side] = sp;
        c.addView(title);

        int bg = rot ? BROWN_L : BLUE_L, fg = rot ? BROWN_T : BLUE_T;
        // cells: {label, axis, dir}; axis -1 = empty, -2 = pad home
        Object[][][] grid = rot ? new Object[][][]{
                {null, {"↺\nRz+", 5, 1}, null},
                {{"↻\nRx−", 3, -1}, {"⌂", -2, 0}, {"↺\nRx+", 3, 1}},
                {{"↻\nRy−", 4, -1}, null, {"↺\nRy+", 4, 1}},
                {null, {"↻\nRz−", 5, -1}, null}}
                : new Object[][][]{
                {null, {"↑\nZ+ 위", 2, 1}, null},
                {{"←\nY+ 좌", 1, 1}, {"⌂", -2, 0}, {"→\nY− 우", 1, -1}},
                {{"↙\nX− 뒤", 0, -1}, null, {"↗\nX+ 앞", 0, 1}},
                {null, {"↓\nZ− 아래", 2, -1}, null}};
        for (Object[][] row : grid) {
            LinearLayout rw = hbox();
            rw.setMotionEventSplittingEnabled(true);
            for (Object[] cell : row) {
                LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(0, -1, 1f);
                lp.setMargins(dp(2), dp(2), dp(2), dp(2));
                if (cell == null) {
                    rw.addView(new View(this), lp);
                    continue;
                }
                int axis = (Integer) cell[1], dir = (Integer) cell[2];
                TextView b = axis == -2 ? button((String) cell[0], GREY_L, MUTED, 18)
                        : button((String) cell[0], bg, fg, 12);
                if (axis == -2) {
                    b.setOnClickListener(v -> ctl.padHome(side, rot));
                } else {
                    attachJog(b, side, rot ? axis + 0 : axis, dir, bg);
                    jogKeys.put(RobotModel.SIDE[side] + ":" + axis + ":" + dir, b);
                }
                rw.addView(b, lp);
            }
            c.addView(rw, new LinearLayout.LayoutParams(-1, 0, 1f));
        }
        return c;
    }

    /** Hold (continuous mode) or tap (step mode) on a jog key. axis 0..5. */
    private void attachJog(final TextView b, final int side, final int axis, final int dir, final int bg) {
        keyResets.add(() -> {
            b.setBackground(round(bg, 10));
            b.setTextColor(axis < 3 ? BLUE_T : BROWN_T);
        });
        b.setOnTouchListener((v, ev) -> {
            int a = ev.getActionMasked();
            if (a == MotionEvent.ACTION_DOWN) {
                b.setBackground(round(axis < 3 ? BLUE : BROWN, 10));
                b.setTextColor(Color.WHITE);
                v.performHapticFeedback(android.view.HapticFeedbackConstants.VIRTUAL_KEY);
                if (cbContinuous.isChecked()) {
                    ctl.setHold(side, axis, dir);
                } else {
                    double amt = axis < 3
                            ? Double.parseDouble(MM[mmIdx[side]]) / 1000.0
                            : Math.toRadians(Double.parseDouble(DEG[degIdx[side]]));
                    ctl.step(side, axis, dir, amt);
                }
            } else if (a == MotionEvent.ACTION_UP || a == MotionEvent.ACTION_CANCEL) {
                b.setBackground(round(bg, 10));
                b.setTextColor(axis < 3 ? BLUE_T : BROWN_T);
                ctl.setHold(side, axis, 0);
            }
            return true;
        });
    }

    private View centerPanel() {
        LinearLayout c = vbox();
        c.setBackground(round(CARD, 14));
        c.setPadding(dp(6), dp(6), dp(6), dp(6));

        c.setMotionEventSplittingEnabled(true);
        robotView = new RobotView(this, model, () -> ctl.snapshotQ());
        c.addView(robotView, new LinearLayout.LayoutParams(-1, 0, 1f));

        LinearLayout sp = vbox();
        sp.setBackground(stroke(CARD, Color.rgb(230, 233, 239), 10));
        sp.setPadding(dp(8), dp(3), dp(8), dp(3));
        sp.addView(text("이동 속도", 12, INK, true));
        sbSpeed = new SeekBar(this);
        sbSpeed.setMax(100);
        sbSpeed.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar s, int v, boolean user) {
                ctl.speed01 = v / 100.0;
                if (!syncingSpeed) {
                    syncingSpeed = true;
                    if (v < 33) rbSlow.setChecked(true);
                    else if (v < 67) rbMid.setChecked(true);
                    else rbFast.setChecked(true);
                    syncingSpeed = false;
                }
            }

            @Override
            public void onStartTrackingTouch(SeekBar s) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar s) {
            }
        });
        sp.addView(sbSpeed);
        LinearLayout lab = hbox();
        TextView l1 = text("느림", 10, MUTED, false), l2 = text("보통", 10, MUTED, false), l3 = text("빠름", 10, MUTED, false);
        l2.setGravity(Gravity.CENTER);
        l3.setGravity(Gravity.END);
        lab.addView(l1, new LinearLayout.LayoutParams(0, -2, 1f));
        lab.addView(l2, new LinearLayout.LayoutParams(0, -2, 1f));
        lab.addView(l3, new LinearLayout.LayoutParams(0, -2, 1f));
        sp.addView(lab);
        LinearLayout.LayoutParams lsp = new LinearLayout.LayoutParams(-1, -2);
        lsp.topMargin = dp(4);
        c.addView(sp, lsp);

        LinearLayout opts = hbox();
        cbContinuous = check("연속 이동 (버튼 유지)");
        cbContinuous.setOnCheckedChangeListener((v, on) -> {
            ctl.continuous = on;
            ctl.releaseAll();
        });
        cbFrames = check("좌표계 표시");
        cbFrames.setOnCheckedChangeListener((v, on) -> robotView.showFrames = on);
        opts.addView(cbContinuous, new LinearLayout.LayoutParams(0, -2, 1.25f));
        opts.addView(cbFrames, new LinearLayout.LayoutParams(0, -2, 1f));
        c.addView(opts);
        return c;
    }

    private View bottomBar() {
        LinearLayout bar = hbox();
        bar.setBackground(round(CARD, 14));
        bar.setPadding(dp(12), dp(4), dp(10), dp(4));
        bar.setGravity(Gravity.CENTER_VERTICAL);
        bar.addView(text("프리셋 동작", 12, INK, true));
        TextView b1 = button("⌂  초기자세", GREY_L, INK, 12);
        b1.setOnClickListener(v -> ctl.moveHome(-1));
        TextView b2 = button("◎  현재 자세 저장", GREY_L, INK, 12);
        b2.setOnClickListener(v -> savePreset());
        TextView b3 = button("▶  저장 자세로 이동", GREY_L, INK, 12);
        b3.setOnClickListener(v -> gotoPreset());
        bar.addView(b1, margins(new LinearLayout.LayoutParams(0, -1, 1f), 10, 0));
        bar.addView(b2, margins(new LinearLayout.LayoutParams(0, -1, 1.3f), 6, 0));
        bar.addView(b3, margins(new LinearLayout.LayoutParams(0, -1, 1.3f), 6, 0));
        bar.addView(divider());
        bar.addView(text("속도 모드", 12, INK, true));
        RadioGroup rg = new RadioGroup(this);
        rg.setOrientation(RadioGroup.HORIZONTAL);
        rbSlow = radio("느림");
        rbMid = radio("보통");
        rbFast = radio("빠름");
        rg.addView(rbSlow);
        rg.addView(rbMid);
        rg.addView(rbFast);
        rg.setOnCheckedChangeListener((g, id) -> {
            if (syncingSpeed) return;
            syncingSpeed = true;
            sbSpeed.setProgress(rbSlow.isChecked() ? 15 : rbFast.isChecked() ? 85 : 50);
            syncingSpeed = false;
        });
        bar.addView(rg, margins(new LinearLayout.LayoutParams(-2, -2), 6, 0));
        bar.addView(divider());
        cbTool = check("툴 좌표계 기준");
        cbTool.setOnCheckedChangeListener((v, on) -> ctl.toolFrame = on);
        bar.addView(cbTool);
        cbMirror = check("두 팔 동시 제어 (대칭)");
        cbMirror.setOnCheckedChangeListener((v, on) -> ctl.mirror = on);
        bar.addView(cbMirror, margins(new LinearLayout.LayoutParams(-2, -2), 6, 0));
        return bar;
    }

    private void savePreset() {
        double[][] q = ctl.snapshotQ();
        StringBuilder sb = new StringBuilder();
        for (int s = 0; s < 2; s++)
            for (int i = 0; i < 7; i++) sb.append(s + i > 0 ? "," : "").append(q[s][i]);
        getSharedPreferences("teleop", MODE_PRIVATE).edit().putString("preset", sb.toString()).apply();
        toast("양팔 자세 저장");
    }

    private void gotoPreset() {
        String s = getSharedPreferences("teleop", MODE_PRIVATE).getString("preset", "");
        if (s.isEmpty()) {
            toast("저장된 자세가 없습니다");
            return;
        }
        String[] t = s.split(",");
        double[][] q = new double[2][7];
        for (int i = 0; i < 14; i++) q[i / 7][i % 7] = Double.parseDouble(t[i]);
        ctl.moveJoints(RA, q[RA]);
        ctl.moveJoints(LA, q[LA]);
    }

    private void showSettings() {
        LinearLayout f = vbox();
        f.setPadding(dp(20), dp(8), dp(20), dp(8));
        EditText eCmd = field(f, "명령 토픽 (발행)", topicCmd);
        EditText eSt = field(f, "상태 토픽 (구독, 시작 자세 동기화)", topicState);
        EditText eHz = field(f, "발행 주기 [Hz]", String.valueOf(pubHz));
        EditText eGain = field(f, "게인 스케일 (비우면 생략 → 브리지 기본 0.1, 값은 (0,1])", gain);
        EditText eId = field(f, "클라이언트 ID", clientId);
        CheckBox cServo = check("서보 플래그 = 1 (실기 구동). 끄면 0 = 서보 off");
        cServo.setChecked(servoFlag);
        f.addView(cServo);
        CheckBox cAllow = check("상태 토픽 없이도 발행 허용 (실기에서는 끄세요)");
        cAllow.setChecked(allowWithoutState);
        f.addView(cAllow);
        CheckBox cPrio = check("위치 우선: 막히면 손끝 자세를 최대 30° 양보");
        cPrio.setChecked(ctl.positionPriority);
        f.addView(cPrio);
        TextView note = text("q는 rmd/제어기 규약 그대로(라디안) 보냅니다. 관절 한계는 실기 측정 가동 범위 ∩ 손목 특이자세 여유.",
                11, MUTED, false);
        f.addView(note);
        ScrollView sv = new ScrollView(this);
        sv.addView(f);
        new AlertDialog.Builder(this)
                .setTitle("설정")
                .setView(sv)
                .setPositiveButton("저장", (d, w) -> {
                    topicCmd = eCmd.getText().toString().trim();
                    topicState = eSt.getText().toString().trim();
                    try {
                        pubHz = Math.max(1, Math.min(100, Integer.parseInt(eHz.getText().toString().trim())));
                    } catch (NumberFormatException e) {
                        pubHz = 50;
                    }
                    String g = eGain.getText().toString().trim();
                    try {
                        double gv = Double.parseDouble(g);
                        gain = gv > 0 && gv <= 1 ? g : "";
                    } catch (NumberFormatException e) {
                        gain = "";
                    }
                    clientId = eId.getText().toString().trim();
                    if (clientId.isEmpty()) clientId = "phone_teleop";
                    servoFlag = cServo.isChecked();
                    allowWithoutState = cAllow.isChecked();
                    ctl.positionPriority = cPrio.isChecked();
                    saveSettings();
                    hideSystemBars();
                })
                .setNegativeButton("취소", (d, w) -> hideSystemBars())
                .show();
    }

    // ------------------------------------------------------------------ prefs

    private void loadSettings() {
        SharedPreferences p = getSharedPreferences("teleop", MODE_PRIVATE);
        topicCmd = p.getString("topicCmd", "/humanoid/upper/cmd/joint");
        topicState = p.getString("topicState", "/humanoid/upper/joint_state");
        pubHz = p.getInt("pubHz", 50);
        gain = p.getString("gain", "");
        clientId = p.getString("clientId", "phone_teleop_" + Build.MODEL.replace(' ', '_'));
        servoFlag = p.getBoolean("servo", false);
        allowWithoutState = p.getBoolean("allowNoState", false);
        ctl.positionPriority = p.getBoolean("posPrio", true);
    }

    private void saveSettings() {
        getSharedPreferences("teleop", MODE_PRIVATE).edit()
                .putString("topicCmd", topicCmd).putString("topicState", topicState).putInt("pubHz", pubHz)
                .putString("gain", gain).putString("clientId", clientId).putBoolean("servo", servoFlag)
                .putBoolean("allowNoState", allowWithoutState).putBoolean("posPrio", ctl.positionPriority).apply();
    }

    private void loadPrefs() {
        SharedPreferences p = getSharedPreferences("teleop", MODE_PRIVATE);
        boolean wifi = p.getBoolean("wifi", false);
        rbUsb.setChecked(!wifi);
        rbWifi.setChecked(wifi);
        etHost.setEnabled(wifi);
        etHost.setText(p.getString("host", ""));
        int pt = p.getInt("port", 1883);
        etPort.setText(String.valueOf(pt == 5001 ? 1883 : pt));          // old jog app used 5001
        sbSpeed.setProgress(p.getInt("speedPct", 50));
        cbContinuous.setChecked(p.getBoolean("continuous", true));
        ctl.continuous = cbContinuous.isChecked();
        cbFrames.setChecked(p.getBoolean("frames", true));
        cbTool.setChecked(p.getBoolean("toolFrame", false));
        for (int s = 0; s < 2; s++) {
            mmIdx[s] = Math.max(0, Math.min(MM.length - 1, p.getInt("mm" + s, 2)));
            degIdx[s] = Math.max(0, Math.min(DEG.length - 1, p.getInt("deg" + s, 2)));
            stepLabelMm[s].setText(MM[mmIdx[s]] + " mm ▾");
            stepLabelDeg[s].setText(DEG[degIdx[s]] + "° ▾");
        }
    }

    private void savePrefs() {
        int pt;
        try {
            pt = Integer.parseInt(etPort.getText().toString().trim());
        } catch (NumberFormatException e) {
            pt = 1883;
        }
        SharedPreferences.Editor e = getSharedPreferences("teleop", MODE_PRIVATE).edit()
                .putBoolean("wifi", rbWifi.isChecked()).putString("host", etHost.getText().toString().trim())
                .putInt("port", pt).putInt("speedPct", sbSpeed.getProgress())
                .putBoolean("continuous", cbContinuous.isChecked()).putBoolean("frames", cbFrames.isChecked())
                .putBoolean("toolFrame", cbTool.isChecked());
        for (int s = 0; s < 2; s++) {
            e.putInt("mm" + s, mmIdx[s]);
            e.putInt("deg" + s, degIdx[s]);
        }
        e.apply();
    }

    // ------------------------------------------------------------------ view helpers

    private LinearLayout vbox() {
        LinearLayout l = new LinearLayout(this);
        l.setOrientation(LinearLayout.VERTICAL);
        return l;
    }

    private LinearLayout hbox() {
        LinearLayout l = new LinearLayout(this);
        l.setOrientation(LinearLayout.HORIZONTAL);
        return l;
    }

    private LinearLayout labeled(String label) {
        LinearLayout l = vbox();
        l.addView(text(label, 10, MUTED, true));
        return l;
    }

    private TextView text(String s, int sp, int color, boolean bold) {
        TextView t = new TextView(this);
        t.setText(s);
        t.setTextSize(sp);
        t.setTextColor(color);
        if (bold) t.setTypeface(Typeface.DEFAULT_BOLD);
        return t;
    }

    private TextView button(String s, int bg, int fg, int sp) {
        TextView t = text(s, sp, fg, true);
        t.setGravity(Gravity.CENTER);
        t.setBackground(round(bg, 10));
        t.setClickable(true);
        t.setLineSpacing(0, 0.9f);
        return t;
    }

    private EditText input(String hint, int type) {
        EditText e = new EditText(this);
        e.setHint(hint);
        e.setInputType(type);
        e.setSingleLine(true);
        e.setTextSize(13);
        e.setPadding(dp(8), 0, dp(8), 0);
        e.setBackground(stroke(CARD, Color.rgb(205, 210, 220), 8));
        return e;
    }

    private EditText field(LinearLayout f, String label, String value) {
        TextView l = text(label, 12, MUTED, true);
        l.setPadding(0, dp(8), 0, dp(2));
        f.addView(l);
        EditText e = new EditText(this);
        e.setText(value);
        e.setSingleLine(true);
        e.setTextSize(14);
        f.addView(e);
        return e;
    }

    private RadioButton radio(String s) {
        RadioButton r = new RadioButton(this);
        r.setText(s);
        r.setTextSize(12);
        r.setId(View.generateViewId());
        return r;
    }

    private CheckBox check(String s) {
        CheckBox c = new CheckBox(this);
        c.setText(s);
        c.setTextSize(11);
        return c;
    }

    private View divider() {
        View v = new View(this);
        v.setBackgroundColor(Color.rgb(226, 229, 235));
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(dp(1), dp(28));
        lp.setMargins(dp(10), 0, dp(10), 0);
        v.setLayoutParams(lp);
        return v;
    }

    private LinearLayout.LayoutParams margins(LinearLayout.LayoutParams lp, int l, int r) {
        lp.leftMargin = dp(l);
        lp.rightMargin = dp(r);
        return lp;
    }

    private GradientDrawable round(int color, int radiusDp) {
        GradientDrawable g = new GradientDrawable();
        g.setColor(color);
        g.setCornerRadius(dp(radiusDp));
        return g;
    }

    private GradientDrawable stroke(int color, int strokeColor, int radiusDp) {
        GradientDrawable g = round(color, radiusDp);
        g.setStroke(dp(1), strokeColor);
        return g;
    }

    private int dp(int v) {
        return Math.round(v * getResources().getDisplayMetrics().density);
    }

    private void toast(String s) {
        Toast.makeText(this, s, Toast.LENGTH_SHORT).show();
    }
}
