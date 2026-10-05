package com.humanoid.teleop;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * One TCP client connection to the PC, reconnecting every second while started.
 * Same code for USB and Wi-Fi: over USB the host is 127.0.0.1 and adb reverse
 * carries the socket through the cable.
 */
final class TeleopLink {

    interface Listener {
        void onLinkState(String text, boolean connected);

        void onPcLine(String line);
    }

    private static final int QUEUE_MAX = 400; // ~2 s at 200 Hz; older samples are useless

    private final Listener listener;
    private final LinkedBlockingQueue<String> queue = new LinkedBlockingQueue<>();
    private volatile boolean running;
    private volatile boolean connected;
    private volatile Socket socket;
    private Thread worker;

    TeleopLink(Listener l) {
        listener = l;
    }

    boolean isRunning() {
        return running;
    }

    boolean isConnected() {
        return connected;
    }

    void send(String line) {
        if (!connected) return;
        if (queue.size() >= QUEUE_MAX) queue.poll();
        queue.offer(line);
    }

    synchronized void start(final String host, final int port, final String hello) {
        stop();
        running = true;
        worker = new Thread(() -> loop(host, port, hello), "teleop-link");
        worker.start();
    }

    synchronized void stop() {
        running = false;
        closeSocket();
        if (worker != null) {
            worker.interrupt();
            worker = null;
        }
    }

    private void closeSocket() {
        Socket s = socket;
        socket = null;
        connected = false;
        if (s != null) {
            try {
                s.close();
            } catch (Exception ignored) {
            }
        }
    }

    private void loop(String host, int port, String hello) {
        while (running) {
            listener.onLinkState("접속 시도 " + host + ":" + port + " ...", false);
            try {
                Socket s = new Socket();
                s.setTcpNoDelay(true);
                s.connect(new InetSocketAddress(host, port), 2000);
                socket = s;
                queue.clear();
                OutputStream out = s.getOutputStream();
                out.write((hello + "\n").getBytes(StandardCharsets.UTF_8));
                out.flush();
                connected = true;
                listener.onLinkState("연결됨 " + host + ":" + port, true);

                Thread reader = new Thread(() -> readLoop(s), "teleop-read");
                reader.start();

                StringBuilder sb = new StringBuilder();
                while (running && !s.isClosed()) {
                    String line = queue.poll(200, TimeUnit.MILLISECONDS);
                    if (line == null) continue;
                    sb.setLength(0);
                    sb.append(line).append('\n');
                    String more;
                    while ((more = queue.poll()) != null) sb.append(more).append('\n'); // batch
                    out.write(sb.toString().getBytes(StandardCharsets.UTF_8));
                    out.flush();
                }
            } catch (InterruptedException e) {
                break;
            } catch (Exception e) {
                if (running) listener.onLinkState("끊김: " + e.getMessage(), false);
            } finally {
                closeSocket();
            }
            if (!running) break;
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                break;
            }
        }
        listener.onLinkState("연결 안 됨", false);
    }

    private void readLoop(Socket s) {
        try {
            BufferedReader in = new BufferedReader(
                    new InputStreamReader(s.getInputStream(), StandardCharsets.UTF_8));
            String line;
            while ((line = in.readLine()) != null) listener.onPcLine(line);
        } catch (Exception ignored) {
        }
        // PC closed the socket: make the writer notice
        try {
            s.close();
        } catch (Exception ignored) {
        }
    }
}
