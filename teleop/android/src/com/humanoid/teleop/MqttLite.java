package com.humanoid.teleop;

import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.charset.StandardCharsets;

/**
 * Minimal MQTT 3.1.1 client: CONNECT (clean session), PUBLISH / SUBSCRIBE at
 * QoS 0, PINGREQ, DISCONNECT.  That is all the teleoperation needs (the robot
 * bridge takes QoS 0 commands), and it keeps the app free of libraries.
 * Not thread-safe for concurrent publishers: publish() is synchronized.
 */
final class MqttLite {

    interface Listener {
        void onMessage(String topic, byte[] payload);

        void onClosed(String reason);
    }

    private final Socket sock = new Socket();
    private OutputStream out;
    private DataInputStream in;
    private volatile boolean open;
    private final int keepAlive;
    private volatile long lastTx;

    MqttLite(int keepAliveSec) {
        keepAlive = keepAliveSec;
    }

    boolean isOpen() {
        return open;
    }

    /** Blocking connect; throws if the broker refuses.  Starts the reader and ping threads. */
    void connect(String host, int port, String clientId, final Listener l) throws IOException {
        sock.setTcpNoDelay(true);
        sock.connect(new InetSocketAddress(host, port), 3000);
        sock.setSoTimeout(5000);
        out = sock.getOutputStream();
        in = new DataInputStream(sock.getInputStream());

        ByteArrayOutputStream vh = new ByteArrayOutputStream();
        str(vh, "MQTT");
        vh.write(4);                         // protocol level 3.1.1
        vh.write(0x02);                      // clean session
        vh.write(keepAlive >> 8);
        vh.write(keepAlive & 0xff);
        str(vh, clientId);
        packet(0x10, vh.toByteArray());

        int type = in.readUnsignedByte();
        int len = readLen();
        byte[] ack = new byte[len];
        in.readFully(ack);
        if ((type & 0xf0) != 0x20 || len < 2 || ack[1] != 0) {
            sock.close();
            throw new IOException("broker refused (CONNACK code " + (len >= 2 ? ack[1] : -1) + ")");
        }
        sock.setSoTimeout(0);
        open = true;

        new Thread(() -> {
            String why = "closed";
            try {
                while (open) {
                    int h = in.readUnsignedByte();
                    int n = readLen();
                    byte[] b = new byte[n];
                    in.readFully(b);
                    if ((h & 0xf0) == 0x30) {                  // PUBLISH
                        int tl = ((b[0] & 0xff) << 8) | (b[1] & 0xff);
                        String topic = new String(b, 2, tl, StandardCharsets.UTF_8);
                        int off = 2 + tl + (((h >> 1) & 3) > 0 ? 2 : 0);
                        byte[] payload = new byte[n - off];
                        System.arraycopy(b, off, payload, 0, payload.length);
                        l.onMessage(topic, payload);
                    }
                }
            } catch (IOException e) {
                why = e.getMessage() == null ? e.toString() : e.getMessage();
            }
            boolean was = open;
            close();
            if (was) l.onClosed(why);
        }, "mqtt-read").start();

        new Thread(() -> {
            while (open) {
                try {
                    Thread.sleep(1000);
                    if (System.currentTimeMillis() - lastTx > keepAlive * 500L) packet(0xC0, new byte[0]);
                } catch (Exception e) {
                    break;
                }
            }
        }, "mqtt-ping").start();
    }

    void subscribe(String topic) throws IOException {
        ByteArrayOutputStream b = new ByteArrayOutputStream();
        b.write(0);
        b.write(1);                          // packet id 1
        str(b, topic);
        b.write(0);                          // QoS 0
        packet(0x82, b.toByteArray());
    }

    void publish(String topic, byte[] payload) throws IOException {
        ByteArrayOutputStream b = new ByteArrayOutputStream(payload.length + topic.length() + 4);
        str(b, topic);
        b.write(payload, 0, payload.length);
        packet(0x30, b.toByteArray());
    }

    void close() {
        if (!open) {
            try {
                sock.close();
            } catch (IOException ignored) {
            }
            return;
        }
        open = false;
        try {
            packet(0xE0, new byte[0]);
        } catch (IOException ignored) {
        }
        try {
            sock.close();
        } catch (IOException ignored) {
        }
    }

    // ------------------------------------------------------------------ framing

    private synchronized void packet(int header, byte[] body) throws IOException {
        ByteArrayOutputStream b = new ByteArrayOutputStream(body.length + 5);
        b.write(header);
        int n = body.length;
        do {
            int d = n % 128;
            n /= 128;
            if (n > 0) d |= 0x80;
            b.write(d);
        } while (n > 0);
        b.write(body, 0, body.length);
        out.write(b.toByteArray());
        out.flush();
        lastTx = System.currentTimeMillis();
    }

    private int readLen() throws IOException {
        int mult = 1, v = 0, d;
        do {
            d = in.readUnsignedByte();
            v += (d & 0x7f) * mult;
            mult *= 128;
        } while ((d & 0x80) != 0 && mult <= 128 * 128 * 128);
        return v;
    }

    private static void str(ByteArrayOutputStream b, String s) {
        byte[] u = s.getBytes(StandardCharsets.UTF_8);
        b.write(u.length >> 8);
        b.write(u.length & 0xff);
        b.write(u, 0, u.length);
    }
}
