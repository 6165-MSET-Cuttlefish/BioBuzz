package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

/**
 * A fake Limelight that outlives a Sloth reload. It is kept in a map that outlives the reload (OpMode.blackboard) as
 * JDK functions only: after a reload, FakeLimelightServer is another class, so the new code can't cast the old server.
 */
final class ReloadFake {
    static final String KEY = "Bench: Limelight Reload fake";

    private final Map<String, Object> board;
    private final Map<String, Object> handle;

    private ReloadFake(Map<String, Object> board, Map<String, Object> handle) {
        this.board = board;
        this.handle = handle;
    }

    /** Puts the server on the board holding every POST until its client gives up; GETs are answered. */
    static ReloadFake start(Map<String, Object> board, FakeLimelightServer server) {
        if (board.containsKey(KEY)) throw new IllegalStateException("a fake Limelight is already on the board");
        server.setMode(FakeLimelightServer.Mode.HANG_POSTS);
        Map<String, Object> handle = new LinkedHashMap<>();
        handle.put("openPosts", (IntSupplier) server::openPosts);
        handle.put("posts", (Supplier<List<Map<String, Object>>>) () -> posts(server));
        handle.put("answer", (Runnable) () -> server.setMode(FakeLimelightServer.Mode.OK));
        handle.put("port", (IntSupplier) server::port);
        handle.put("check", (Runnable) server::check);
        handle.put("close", (Runnable) server::close);
        board.put(KEY, handle);
        return new ReloadFake(board, handle);
    }

    /** The fake an earlier run left on the board, whichever Sloth load built it; null when there is none. */
    @SuppressWarnings("unchecked")
    static ReloadFake carried(Map<String, Object> board) {
        Object handle = board.get(KEY);
        return handle == null ? null : new ReloadFake(board, (Map<String, Object>) handle);
    }

    int openPosts() {
        return ((IntSupplier) handle.get("openPosts")).getAsInt();
    }

    /** Every POST and DELETE so far: line, held, receivedNs, endedNs (0 while open) and endedBy, on System.nanoTime(). */
    @SuppressWarnings("unchecked")
    List<Map<String, Object>> posts() {
        return ((Supplier<List<Map<String, Object>>>) handle.get("posts")).get();
    }

    /** From now on POSTs are answered; one already held stays held until its client gives up. */
    void answer() {
        ((Runnable) handle.get("answer")).run();
    }

    int port() {
        return ((IntSupplier) handle.get("port")).getAsInt();
    }

    void check() {
        ((Runnable) handle.get("check")).run();
    }

    void close() {
        try {
            ((Runnable) handle.get("close")).run();
        } finally {
            board.remove(KEY, handle);
        }
    }

    double longestOpenPostMs(long nowNs) {
        double longest = 0;
        for (Map<String, Object> p : posts()) {
            if ((Long) p.get("endedNs") == 0) longest = Math.max(longest, (nowNs - (Long) p.get("receivedNs")) / 1e6);
        }
        return longest;
    }

    private static List<Map<String, Object>> posts(FakeLimelightServer server) {
        List<Map<String, Object>> list = new ArrayList<>();
        for (FakeLimelightServer.Request r : server.posts()) {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("line", r.line);
            m.put("held", r.held);
            m.put("receivedNs", r.receivedNs);
            m.put("endedNs", r.endedNs());
            m.put("endedBy", r.endedBy());
            list.add(m);
        }
        return list;
    }
}
