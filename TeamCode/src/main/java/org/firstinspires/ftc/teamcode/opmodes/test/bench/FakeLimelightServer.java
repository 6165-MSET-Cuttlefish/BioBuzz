package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import android.os.Debug;

import org.firstinspires.ftc.teamcode.biobuzz.modules.vision.LimelightSync;

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
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.function.LongSupplier;

final class FakeLimelightServer {
    /** HANG_POSTS answers every GET and holds every POST and DELETE: a Limelight serving results that won't take a command. */
    enum Mode { OK, ERROR_500, HANG, HANG_POSTS }

    /** A POST or DELETE as the fake saw it, on System.nanoTime(); a held one ends when its client gives up. */
    static final class Request {
        final String line;
        final long receivedNs;
        final boolean held;
        private volatile String endedBy;
        private volatile long endedNs;

        Request(String line, long receivedNs, boolean held) {
            this.line = line;
            this.receivedNs = receivedNs;
            this.held = held;
        }

        /** 0 while open. */
        long endedNs() {
            return endedNs;
        }

        String endedBy() {
            return endedBy;
        }

        private void end(String by) {
            endedBy = by;
            endedNs = System.nanoTime();
        }
    }

    static final int PORT = 15807;
    private static final String PIPELINE_SWITCH = "POST /pipeline-switch?index=";

    private final ServerSocket server;
    private final Thread thread;
    private final List<Socket> held = new ArrayList<>();
    private final List<Request> posts = new ArrayList<>();
    private final int balls;
    private final double scriptId;
    private final LongSupplier threadCpuNanos;
    private final long bootNanos = System.nanoTime();
    private volatile Mode mode = Mode.OK;
    private volatile int pipeline = 4;
    private volatile boolean closing;
    private volatile long requests;
    private volatile long cpuNanos;
    private volatile Throwable failure;

    FakeLimelightServer(int balls) {
        this(balls, LimelightSync.FILE_SCRIPT_ID);
    }

    /** Reports {@code scriptId} in llpython[0], and as pID the pipeline of the last switch it answered. */
    FakeLimelightServer(int balls, double scriptId) {
        this(PORT, balls, scriptId, Debug::threadCpuTimeNanos);
    }

    /** Port 0 takes a free port; off a hub, threadCpuNanos stands in for android.os.Debug's per-thread CPU clock. */
    FakeLimelightServer(int port, int balls, double scriptId, LongSupplier threadCpuNanos) {
        this.balls = balls;
        this.scriptId = scriptId;
        this.threadCpuNanos = threadCpuNanos;
        try {
            server = new ServerSocket();
            server.setReuseAddress(true);
            server.bind(new java.net.InetSocketAddress(InetAddress.getByName("127.0.0.1"), port));
        } catch (IOException e) {
            throw new IllegalStateException("can't listen on 127.0.0.1:" + port + " for the fake Limelight", e);
        }
        thread = new Thread(this::serve, "bench-fake-limelight");
        thread.start();
    }

    void setMode(Mode mode) {
        this.mode = mode;
    }

    int port() {
        return server.getLocalPort();
    }

    long requests() {
        return requests;
    }

    double cpuMs() {
        return cpuNanos / 1e6;
    }

    List<Request> posts() {
        synchronized (posts) {
            return new ArrayList<>(posts);
        }
    }

    int openPosts() {
        int open = 0;
        synchronized (posts) {
            for (Request r : posts) if (r.endedNs == 0) open++;
        }
        return open;
    }

    /** Rethrows whatever stopped the server thread, so a broken fake can't pass for a slow Limelight. */
    void check() {
        Throwable t = failure;
        if (t != null) throw new IllegalStateException("fake Limelight server failed", t);
    }

    void close() {
        closing = true;
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
                long cpu0 = threadCpuNanos.getAsLong();
                handle(s);
                cpuNanos += threadCpuNanos.getAsLong() - cpu0;
                requests++;
            }
        } catch (Throwable t) {
            failure = t;
        }
    }

    private void handle(Socket s) throws IOException {
        String request = readRequest(s.getInputStream());
        if (request.isEmpty()) {
            s.close();
            return;
        }
        Mode m = mode;
        boolean get = request.startsWith("GET ");
        boolean hold = m == Mode.HANG || (m == Mode.HANG_POSTS && !get);
        Request logged = get ? null : log(request, hold);
        if (hold) {
            synchronized (held) {
                held.add(s);
            }
            if (logged != null) watch(s, logged);
            return;
        }
        try {
            OutputStream out = s.getOutputStream();
            if (m == Mode.ERROR_500) {
                out.write("HTTP/1.1 500 Internal Server Error\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"
                        .getBytes(StandardCharsets.US_ASCII));
            } else {
                if (request.startsWith(PIPELINE_SWITCH)) {
                    pipeline = Integer.parseInt(request.substring(PIPELINE_SWITCH.length(),
                            request.indexOf(' ', PIPELINE_SWITCH.length())));
                }
                String body = request.startsWith("GET /results")
                        ? ParsedLLResult.synthetic(balls, 0, 0, (System.nanoTime() - bootNanos) / 1e6, pipeline,
                                scriptId)
                        : request.startsWith("GET /status")
                        ? "{\"cid\":0,\"cpu\":31.5,\"finalYaw\":0,\"fps\":90,\"hwType\":3,\"name\":\"fake\","
                        + "\"pipeImgCount\":0,\"pipelineIndex\":" + pipeline + ",\"pipelineType\":\"pipe_python\","
                        + "\"ram\":40.2,\"snapshotMode\":0,\"temp\":45.1}"
                        : "{}";
                byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
                byte[] head = ("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: " + bytes.length
                        + "\r\nConnection: close\r\n\r\n").getBytes(StandardCharsets.US_ASCII);
                byte[] response = Arrays.copyOf(head, head.length + bytes.length);
                System.arraycopy(bytes, 0, response, head.length, bytes.length);
                // One write: a second write to a client that already gave up throws EPIPE and would kill the fake.
                out.write(response);
            }
            out.flush();
        } finally {
            s.close();
            if (logged != null) logged.end(m == Mode.ERROR_500 ? "answered 500" : "answered 200");
        }
    }

    private Request log(String request, boolean hold) {
        int eol = request.indexOf('\r');
        Request r = new Request(eol < 0 ? request : request.substring(0, eol), System.nanoTime(), hold);
        synchronized (posts) {
            posts.add(r);
        }
        return r;
    }

    // The client closes its end when its read timeout fires, so this EOF is when the hung call returned.
    private void watch(Socket s, Request r) {
        Thread watcher = new Thread(() -> {
            try {
                String by = "client closed";
                try {
                    InputStream in = s.getInputStream();
                    while (in.read() >= 0) {
                    }
                } catch (SocketException e) {
                    by = "client reset: " + e.getMessage();
                }
                r.end(closing ? "fake closed" : by);
                synchronized (held) {
                    held.remove(s);
                }
                s.close();
            } catch (Throwable t) {
                failure = t;
            }
        }, "bench-fake-limelight-held");
        watcher.setDaemon(true);
        watcher.start();
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
