package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import android.os.Debug;

import java.io.BufferedInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * A stand-in for a Limelight's HTTP API on 127.0.0.1, so the SDK's real Limelight3A poller and blocking
 * calls can be measured on a bare hub. Its own CPU time is counted so it can be taken out of the load figures.
 */
final class FakeLimelightServer {
    enum Mode { OK, ERROR_500, HANG }

    static final int PORT = 15807;

    private final ServerSocket server;
    private final Thread thread;
    private final List<Socket> held = new ArrayList<>();
    private final int balls;
    private final long bootNanos = System.nanoTime();
    private volatile Mode mode = Mode.OK;
    private volatile long requests;
    private volatile long cpuNanos;
    private volatile Throwable failure;

    FakeLimelightServer(int balls) {
        this.balls = balls;
        try {
            server = new ServerSocket();
            server.setReuseAddress(true);
            server.bind(new java.net.InetSocketAddress(InetAddress.getByName("127.0.0.1"), PORT));
        } catch (IOException e) {
            throw new IllegalStateException("can't listen on 127.0.0.1:" + PORT + " for the fake Limelight", e);
        }
        thread = new Thread(this::serve, "bench-fake-limelight");
        thread.start();
    }

    void setMode(Mode mode) {
        this.mode = mode;
    }

    long requests() {
        return requests;
    }

    double cpuMs() {
        return cpuNanos / 1e6;
    }

    /** Rethrows whatever stopped the server thread, so a broken fake can't pass for a slow Limelight. */
    void check() {
        Throwable t = failure;
        if (t != null) throw new IllegalStateException("fake Limelight server failed", t);
    }

    void close() {
        try {
            server.close();
        } catch (IOException e) {
            throw new IllegalStateException("can't close the fake Limelight server", e);
        } finally {
            synchronized (held) {
                for (Socket s : held) closeHeld(s);
                held.clear();
            }
            thread.interrupt();
        }
        try {
            thread.join(2000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void serve() {
        try {
            while (!server.isClosed()) {
                Socket s;
                try {
                    s = server.accept();
                } catch (SocketException closed) {
                    return;
                }
                long cpu0 = Debug.threadCpuTimeNanos();
                handle(s);
                cpuNanos += Debug.threadCpuTimeNanos() - cpu0;
                requests++;
            }
        } catch (Throwable t) {
            failure = t;
        }
    }

    private void handle(Socket s) throws IOException {
        String request = readRequest(s.getInputStream());
        Mode m = mode;
        if (m == Mode.HANG) {
            synchronized (held) {
                held.add(s);
            }
            return;
        }
        try {
            OutputStream out = s.getOutputStream();
            if (m == Mode.ERROR_500) {
                out.write("HTTP/1.1 500 Internal Server Error\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"
                        .getBytes(StandardCharsets.US_ASCII));
            } else {
                String body = request.startsWith("GET /results")
                        ? ParsedLLResult.synthetic(balls, (System.nanoTime() - bootNanos) / 1e6)
                        : request.startsWith("GET /status")
                        ? "{\"cid\":0,\"cpu\":31.5,\"finalYaw\":0,\"fps\":90,\"hwType\":3,\"name\":\"fake\",\"pipeImgCount\":0,"
                        + "\"pipelineIndex\":4,\"pipelineType\":\"pipe_python\",\"ram\":40.2,\"snapshotMode\":0,\"temp\":45.1}"
                        : "{}";
                byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
                out.write(("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: " + bytes.length
                        + "\r\nConnection: close\r\n\r\n").getBytes(StandardCharsets.US_ASCII));
                out.write(bytes);
            }
            out.flush();
        } finally {
            s.close();
        }
    }

    /** The request line and headers, plus any Content-Length body, so the client's write never blocks. */
    private static String readRequest(InputStream raw) throws IOException {
        InputStream in = new BufferedInputStream(raw, 1024);
        StringBuilder head = new StringBuilder();
        int last4 = 0;
        while (last4 != 0x0d0a0d0a) {
            int c = in.read();
            if (c < 0) break;
            head.append((char) c);
            last4 = (last4 << 8) | c;
        }
        String h = head.toString();
        int at = h.toLowerCase(Locale.ROOT).indexOf("content-length:");
        if (at >= 0) {
            int end = h.indexOf('\r', at);
            int length = Integer.parseInt(h.substring(at + 15, end).trim());
            for (int i = 0; i < length && in.read() >= 0; i++) {
                // discard the body
            }
        }
        return h;
    }

    private static void closeHeld(Socket s) {
        try {
            s.close();
        } catch (IOException e) {
            throw new IllegalStateException("can't close a held fake-Limelight connection", e);
        }
    }
}
