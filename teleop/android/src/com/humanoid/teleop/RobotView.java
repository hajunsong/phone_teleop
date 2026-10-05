package com.humanoid.teleop;

import android.content.Context;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.opengl.Matrix;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;

import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import javax.microedition.khronos.egl.EGL10;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.egl.EGLDisplay;
import javax.microedition.khronos.opengles.GL10;

/**
 * Live 3D view of the actual robot: the CAD meshes of the MuJoCo model
 * (assets/robot_mesh.bin) placed by the phone's own forward kinematics at the
 * commanded q.  One-finger drag orbits, pinch zooms.  Both arms are tinted
 * (LA blue, RA brown), and tool frames are drawn when enabled.
 *
 * Default view is from behind the robot, so the robot's left arm (LA) is on
 * the left of the screen, matching the jog panels.
 */
final class RobotView extends GLSurfaceView {

    interface Source {
        /** Current joint vector q[side][7]; called on the GL thread. */
        double[][] viewQ();
    }

    private final RobotModel model;
    private final Source source;
    private final Renderer renderer = new Renderer();
    volatile int selected = -1;                                // -1: both arms highlighted
    volatile boolean showFrames = true;

    private float yaw = -150f, pitch = 15f, dist = 1.7f;       // camera orbit (deg, m)
    private float lastX, lastY;
    private final ScaleGestureDetector scaler;

    RobotView(Context ctx, RobotModel model, Source source) {
        super(ctx);
        this.model = model;
        this.source = source;
        setEGLContextClientVersion(2);
        setEGLConfigChooser(new MsaaChooser());
        setRenderer(renderer);
        setRenderMode(RENDERMODE_CONTINUOUSLY);
        scaler = new ScaleGestureDetector(ctx, new ScaleGestureDetector.SimpleOnScaleGestureListener() {
            @Override
            public boolean onScale(ScaleGestureDetector d) {
                dist = Math.max(0.4f, Math.min(3f, dist / d.getScaleFactor()));
                return true;
            }
        });
    }

    @Override
    public boolean onTouchEvent(MotionEvent e) {
        scaler.onTouchEvent(e);
        if (e.getPointerCount() == 1) {
            switch (e.getActionMasked()) {
                case MotionEvent.ACTION_DOWN:
                    lastX = e.getX();
                    lastY = e.getY();
                    break;
                case MotionEvent.ACTION_MOVE:
                    yaw -= (e.getX() - lastX) * 0.4f;
                    pitch = Math.max(-80, Math.min(80, pitch + (e.getY() - lastY) * 0.4f));
                    lastX = e.getX();
                    lastY = e.getY();
                    break;
            }
        } else {
            lastX = e.getX(0);
            lastY = e.getY(0);
        }
        getParent().requestDisallowInterceptTouchEvent(true);
        return true;
    }

    void resetView() {
        yaw = -150f;
        pitch = 15f;
        dist = 1.7f;
    }

    // ------------------------------------------------------------------ renderer

    private final class Renderer implements GLSurfaceView.Renderer {
        int prog, aPos, aNrm, uMvp, uNrm, uColor;
        int[] vbo;
        int[] count;
        int boxVbo;
        final float[] proj = new float[16], view = new float[16], vp = new float[16],
                mdl = new float[16], mvp = new float[16], nrm = new float[9], tmp = new float[16];
        double[][] pos, rot;

        @Override
        public void onSurfaceCreated(GL10 gl, EGLConfig cfg) {
            prog = program(
                    "uniform mat4 uMvp; uniform mat3 uNrm; attribute vec3 aPos; attribute vec3 aNrm;" +
                            "varying vec3 vN; void main(){ vN = uNrm * aNrm; gl_Position = uMvp * vec4(aPos,1.0); }",
                    "precision mediump float; uniform vec3 uColor; varying vec3 vN;" +
                            "void main(){ vec3 n = normalize(vN);" +
                            " float d1 = abs(dot(n, normalize(vec3(-0.5, 0.4, 0.8))));" +
                            " float d2 = abs(dot(n, normalize(vec3(0.6, -0.5, 0.3))));" +
                            " gl_FragColor = vec4(uColor * (0.38 + 0.55 * d1 + 0.22 * d2), 1.0); }");
            aPos = GLES20.glGetAttribLocation(prog, "aPos");
            aNrm = GLES20.glGetAttribLocation(prog, "aNrm");
            uMvp = GLES20.glGetUniformLocation(prog, "uMvp");
            uNrm = GLES20.glGetUniformLocation(prog, "uNrm");
            uColor = GLES20.glGetUniformLocation(prog, "uColor");
            loadMeshes();
            boxVbo = upload(boxMesh());
            GLES20.glEnable(GLES20.GL_DEPTH_TEST);
            GLES20.glClearColor(0.955f, 0.962f, 0.975f, 1f);
            int n = model.bodies.size();
            pos = new double[n][];
            rot = new double[n][];
        }

        @Override
        public void onSurfaceChanged(GL10 gl, int w, int h) {
            GLES20.glViewport(0, 0, w, h);
            Matrix.perspectiveM(proj, 0, 35f, (float) w / Math.max(1, h), 0.05f, 10f);
        }

        @Override
        public void onDrawFrame(GL10 gl) {
            GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);
            double[][] q = source.viewQ();
            if (q == null || vbo == null) return;
            model.fkAll(q, pos, rot);

            // camera orbits a point between the shoulders; world z is up
            float cx = 0.10f, cy = 0f, cz = 0.20f;
            double ya = Math.toRadians(yaw), pa = Math.toRadians(pitch);
            float ex = cx + (float) (dist * Math.cos(pa) * Math.cos(ya));
            float ey = cy + (float) (dist * Math.cos(pa) * Math.sin(ya));
            float ez = cz + (float) (dist * Math.sin(pa));
            Matrix.setLookAtM(view, 0, ex, ey, ez, cx, cy, cz, 0, 0, 1);
            Matrix.multiplyMM(vp, 0, proj, 0, view, 0);

            GLES20.glUseProgram(prog);
            int sel = selected;
            double[] R = new double[9], p = new double[3];
            for (RobotModel.Geom g : model.geoms) {
                if (g.mesh < 0 || g.mesh >= vbo.length) continue;
                RobotModel.mul(rot[g.body], g.rot, R);
                RobotModel.mulv(rot[g.body], g.pos, p);
                p[0] += pos[g.body][0];
                p[1] += pos[g.body][1];
                p[2] += pos[g.body][2];
                float[] c = colorOf(g.body, sel);
                draw(vbo[g.mesh], count[g.mesh], R, p, 1, 1, 1, c[0], c[1], c[2]);
            }
            if (showFrames) {
                double[] tp = new double[3], tR = new double[9];
                for (int s = 0; s < 2; s++) {
                    model.tool(s, pos, rot, tp, tR);
                    axes(tp, tR, 0.07f);
                }
                axes(new double[]{0, 0, 0}, RobotModel.eye(), 0.10f);      // base frame
            }
        }

        private float[] colorOf(int body, int sel) {
            int side = sideOfBody(body);
            if (side < 0) return new float[]{0.80f, 0.81f, 0.84f};
            boolean on = sel < 0 || side == sel;
            if (side == RobotModel.LA) return on ? new float[]{0.36f, 0.56f, 0.93f} : new float[]{0.66f, 0.73f, 0.86f};
            return on ? new float[]{0.84f, 0.52f, 0.25f} : new float[]{0.86f, 0.74f, 0.64f};
        }

        private int sideOfBody(int b) {
            while (b > 0) {
                RobotModel.Body bb = model.bodies.get(b);
                if (bb.kind != 0) return bb.side;
                if (bb.parent == 0) break;
                b = bb.parent;
            }
            // arm bases (body0) are fixed bodies under base: decide by their children
            for (int i = 1; i < model.bodies.size(); i++) {
                RobotModel.Body c = model.bodies.get(i);
                if (c.parent == b && c.kind != 0) return c.side;
            }
            return -1;
        }

        private void axes(double[] p, double[] R, float len) {
            float[][] col = {{0.90f, 0.20f, 0.20f}, {0.20f, 0.72f, 0.25f}, {0.20f, 0.40f, 0.95f}};
            for (int k = 0; k < 3; k++) {
                // unit box spans x in [0,1]; rotate it onto axis k of R
                double[] A = new double[9];
                for (int r = 0; r < 3; r++) {
                    A[r * 3] = R[r * 3 + k];
                    A[r * 3 + 1] = R[r * 3 + (k + 1) % 3];
                    A[r * 3 + 2] = R[r * 3 + (k + 2) % 3];
                }
                draw(boxVbo, 36, A, p, len, 0.006f, 0.006f, col[k][0], col[k][1], col[k][2]);
            }
        }

        private void draw(int buf, int n, double[] R, double[] p, float sx, float sy, float sz,
                          float r, float g, float b) {
            // column-major model matrix = [R*S | p]
            mdl[0] = (float) R[0] * sx; mdl[4] = (float) R[1] * sy; mdl[8] = (float) R[2] * sz; mdl[12] = (float) p[0];
            mdl[1] = (float) R[3] * sx; mdl[5] = (float) R[4] * sy; mdl[9] = (float) R[5] * sz; mdl[13] = (float) p[1];
            mdl[2] = (float) R[6] * sx; mdl[6] = (float) R[7] * sy; mdl[10] = (float) R[8] * sz; mdl[14] = (float) p[2];
            mdl[3] = 0; mdl[7] = 0; mdl[11] = 0; mdl[15] = 1;
            Matrix.multiplyMM(mvp, 0, vp, 0, mdl, 0);
            nrm[0] = (float) R[0]; nrm[3] = (float) R[1]; nrm[6] = (float) R[2];
            nrm[1] = (float) R[3]; nrm[4] = (float) R[4]; nrm[7] = (float) R[5];
            nrm[2] = (float) R[6]; nrm[5] = (float) R[7]; nrm[8] = (float) R[8];
            GLES20.glUniformMatrix4fv(uMvp, 1, false, mvp, 0);
            GLES20.glUniformMatrix3fv(uNrm, 1, false, nrm, 0);
            GLES20.glUniform3f(uColor, r, g, b);
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, buf);
            GLES20.glEnableVertexAttribArray(aPos);
            GLES20.glEnableVertexAttribArray(aNrm);
            GLES20.glVertexAttribPointer(aPos, 3, GLES20.GL_FLOAT, false, 24, 0);
            GLES20.glVertexAttribPointer(aNrm, 3, GLES20.GL_FLOAT, false, 24, 12);
            GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, n);
        }

        /** Flat-shaded triangles: x y z nx ny nz per vertex. */
        private void loadMeshes() {
            try (InputStream raw = getContext().getAssets().open("robot_mesh.bin")) {
                DataInputStream in = new DataInputStream(new java.io.BufferedInputStream(raw, 1 << 16));
                int nm = le(in);
                vbo = new int[nm];
                count = new int[nm];
                for (int i = 0; i < nm; i++) {
                    int nv = le(in), nf = le(in);
                    float[] v = new float[nv * 3];
                    for (int k = 0; k < v.length; k++) v[k] = Float.intBitsToFloat(le(in));
                    float[] out = new float[nf * 18];
                    int o = 0;
                    for (int f = 0; f < nf; f++) {
                        int a = le(in) * 3, b = le(in) * 3, c = le(in) * 3;
                        float ux = v[b] - v[a], uy = v[b + 1] - v[a + 1], uz = v[b + 2] - v[a + 2];
                        float wx = v[c] - v[a], wy = v[c + 1] - v[a + 1], wz = v[c + 2] - v[a + 2];
                        float nx = uy * wz - uz * wy, ny = uz * wx - ux * wz, nz = ux * wy - uy * wx;
                        float l = (float) Math.sqrt(nx * nx + ny * ny + nz * nz);
                        if (l > 0) { nx /= l; ny /= l; nz /= l; }
                        for (int idx : new int[]{a, b, c}) {
                            out[o++] = v[idx]; out[o++] = v[idx + 1]; out[o++] = v[idx + 2];
                            out[o++] = nx; out[o++] = ny; out[o++] = nz;
                        }
                    }
                    vbo[i] = upload(out);
                    count[i] = nf * 3;
                }
            } catch (IOException e) {
                vbo = null;
            }
        }

        private int le(DataInputStream in) throws IOException {
            return Integer.reverseBytes(in.readInt());
        }

        private int upload(float[] data) {
            FloatBuffer fb = ByteBuffer.allocateDirect(data.length * 4).order(ByteOrder.nativeOrder()).asFloatBuffer();
            fb.put(data).position(0);
            int[] id = new int[1];
            GLES20.glGenBuffers(1, id, 0);
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, id[0]);
            GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, data.length * 4, fb, GLES20.GL_STATIC_DRAW);
            return id[0];
        }

        private float[] boxMesh() {
            // box x in [0,1], y,z in [-1,1] (scaled at draw time), flat normals
            float[][] f = {
                    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
            float[] out = new float[36 * 6];
            int o = 0;
            for (float[] n : f) {
                // two triangles on the face with normal n
                float[][] c = faceCorners(n);
                int[] tri = {0, 1, 2, 0, 2, 3};
                for (int t : tri) {
                    out[o++] = c[t][0]; out[o++] = c[t][1]; out[o++] = c[t][2];
                    out[o++] = n[0]; out[o++] = n[1]; out[o++] = n[2];
                }
            }
            return out;
        }

        private float[][] faceCorners(float[] n) {
            float[][] c = new float[4][3];
            int ax = n[0] != 0 ? 0 : n[1] != 0 ? 1 : 2;
            int u = (ax + 1) % 3, w = (ax + 2) % 3;
            float[][] uv = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
            for (int i = 0; i < 4; i++) {
                c[i][ax] = n[ax];
                c[i][u] = uv[i][0];
                c[i][w] = uv[i][1];
                if (n[ax] < 0) c[i][u] = -c[i][u];
            }
            for (float[] p : c) p[0] = (p[0] + 1) / 2;                 // x in [0,1]
            return c;
        }

        private int program(String vs, String fs) {
            int v = shader(GLES20.GL_VERTEX_SHADER, vs), f = shader(GLES20.GL_FRAGMENT_SHADER, fs);
            int p = GLES20.glCreateProgram();
            GLES20.glAttachShader(p, v);
            GLES20.glAttachShader(p, f);
            GLES20.glLinkProgram(p);
            return p;
        }

        private int shader(int type, String src) {
            int s = GLES20.glCreateShader(type);
            GLES20.glShaderSource(s, src);
            GLES20.glCompileShader(s);
            return s;
        }
    }

    /** 4x MSAA if available, otherwise plain RGB888 + depth. */
    private static final class MsaaChooser implements EGLConfigChooser {
        @Override
        public EGLConfig chooseConfig(EGL10 egl, EGLDisplay d) {
            int[][] attempts = {
                    {EGL10.EGL_RED_SIZE, 8, EGL10.EGL_GREEN_SIZE, 8, EGL10.EGL_BLUE_SIZE, 8, EGL10.EGL_DEPTH_SIZE, 16,
                            EGL10.EGL_RENDERABLE_TYPE, 4, EGL10.EGL_SAMPLE_BUFFERS, 1, EGL10.EGL_SAMPLES, 4, EGL10.EGL_NONE},
                    {EGL10.EGL_RED_SIZE, 8, EGL10.EGL_GREEN_SIZE, 8, EGL10.EGL_BLUE_SIZE, 8, EGL10.EGL_DEPTH_SIZE, 16,
                            EGL10.EGL_RENDERABLE_TYPE, 4, EGL10.EGL_NONE}};
            for (int[] a : attempts) {
                EGLConfig[] cfg = new EGLConfig[1];
                int[] n = new int[1];
                if (egl.eglChooseConfig(d, a, cfg, 1, n) && n[0] > 0) return cfg[0];
            }
            throw new IllegalStateException("no EGL config");
        }
    }
}
