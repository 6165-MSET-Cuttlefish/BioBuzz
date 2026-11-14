package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Test;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.net.ConnectException;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;

/** A fake Limelight built by one class loader and driven through a shared board by a second, as after a deploySloth. */
public class ReloadFakeTest {
    private static final String TEAMCODE = "org.firstinspires.ftc.teamcode.";
    private static final String BENCH = TEAMCODE + "opmodes.test.bench.";

    /** Defines the team's classes itself, as each Sloth load does, and leaves every other class to its parent. */
    private static final class Load extends ClassLoader {
        Load() {
            super(ReloadFakeTest.class.getClassLoader());
        }

        @Override
        protected Class<?> loadClass(String name, boolean resolve) throws ClassNotFoundException {
            if (!name.startsWith(TEAMCODE)) return super.loadClass(name, resolve);
            synchronized (this) {
                Class<?> c = findLoadedClass(name);
                if (c == null) {
                    byte[] bytes = read(name.replace('.', '/') + ".class");
                    c = defineClass(name, bytes, 0, bytes.length);
                }
                if (resolve) resolveClass(c);
                return c;
            }
        }

        private byte[] read(String resource) throws ClassNotFoundException {
            try (InputStream in = getParent().getResourceAsStream(resource)) {
                if (in == null) throw new ClassNotFoundException(resource);
                ByteArrayOutputStream out = new ByteArrayOutputStream();
                byte[] buf = new byte[8192];
                for (int n; (n = in.read(buf)) > 0; ) out.write(buf, 0, n);
                return out.toByteArray();
            } catch (IOException e) {
                throw new ClassNotFoundException(resource, e);
            }
        }
    }

    private static Object call(Object target, String method, Object... args) throws Exception {
        for (Method m : target.getClass().getDeclaredMethods()) {
            if (m.getName().equals(method) && m.getParameterCount() == args.length) {
                m.setAccessible(true);
                return m.invoke(target, args);
            }
        }
        throw new NoSuchMethodException(method);
    }

    private static Object callStatic(Class<?> type, String method, Class<?>[] types, Object... args) throws Exception {
        Method m = type.getDeclaredMethod(method, types);
        m.setAccessible(true);
        return m.invoke(null, args);
    }

    @SuppressWarnings("unchecked")
    private static List<Map<String, Object>> posts(Object fake) throws Exception {
        return (List<Map<String, Object>>) call(fake, "posts");
    }

    private static void waitFor(String what, BooleanSupplier condition) throws InterruptedException {
        long deadline = System.nanoTime() + 3_000_000_000L;
        while (!condition.getAsBoolean()) {
            if (System.nanoTime() > deadline) fail("timed out waiting for " + what);
            Thread.sleep(10);
        }
    }

    private static Socket post(int port, String path) throws IOException {
        Socket s = new Socket("127.0.0.1", port);
        s.getOutputStream().write(("POST " + path + " HTTP/1.1\r\nHost: 127.0.0.1\r\nContent-Length: 2\r\n\r\n{}")
                .getBytes(StandardCharsets.US_ASCII));
        s.getOutputStream().flush();
        return s;
    }

    private static int openPosts(Object fake) {
        try {
            return (Integer) call(fake, "openPosts");
        } catch (Exception e) {
            throw new IllegalStateException(e);
        }
    }

    private static long endedNs(Object fake, int index) {
        try {
            List<Map<String, Object>> all = posts(fake);
            return all.size() > index ? (Long) all.get(index).get("endedNs") : 0;
        } catch (Exception e) {
            throw new IllegalStateException(e);
        }
    }

    @Test
    public void aFakeHeldOverAReloadIsDrivenByTheNewCodeThroughTheBoard() throws Exception {
        Map<String, Object> board = new HashMap<>();
        Load before = new Load();
        Load after = new Load();
        Class<?> serverBefore = before.loadClass(BENCH + "FakeLimelightServer");
        Class<?> serverAfter = after.loadClass(BENCH + "FakeLimelightServer");
        assertNotSame("each load defines its own FakeLimelightServer", serverBefore, serverAfter);

        Constructor<?> make = serverBefore.getDeclaredConstructor(int.class, int.class, double.class, LongSupplier.class);
        make.setAccessible(true);
        Object server = make.newInstance(0, 0, 8388609.0, (LongSupplier) () -> 0L);
        Object startedBy = null;
        try {
            startedBy = callStatic(before.loadClass(BENCH + "ReloadFake"), "start",
                    new Class<?>[] {Map.class, serverBefore}, board, server);
            assertFalse("the reloaded code can't cast the old server, so the board holds JDK types only",
                    serverAfter.isInstance(server));
            assertTrue(board.get("Bench: Limelight Reload fake") instanceof Map);

            Class<?> fakeAfter = after.loadClass(BENCH + "ReloadFake");
            Object carried = callStatic(fakeAfter, "carried", new Class<?>[] {Map.class}, board);
            assertTrue(fakeAfter.isInstance(carried));
            int port = (Integer) call(carried, "port");

            try (Socket hung = post(port, "/pipeline-switch?index=0")) {
                waitFor("the fake to log the POST", () -> openPosts(carried) == 1);
                hung.setSoTimeout(300);
                try {
                    int b = hung.getInputStream().read();
                    fail("the hanging fake answered a POST (read " + b + ")");
                } catch (SocketTimeoutException expected) {
                    assertEquals("held while its client waits", 1, openPosts(carried));
                }
            }
            waitFor("the held POST to end when its client gives up", () -> endedNs(carried, 0) != 0);
            Map<String, Object> first = posts(carried).get(0);
            assertEquals("POST /pipeline-switch?index=0 HTTP/1.1", first.get("line"));
            assertEquals(Boolean.TRUE, first.get("held"));
            assertTrue(String.valueOf(first.get("endedBy")).startsWith("client"));
            assertTrue((Long) first.get("endedNs") > (Long) first.get("receivedNs"));

            call(carried, "answer");
            try (Socket answered = post(port, "/pipeline-switch?index=4")) {
                answered.setSoTimeout(2000);
                byte[] head = new byte[12];
                int n = 0;
                for (int r; n < head.length && (r = answered.getInputStream().read(head, n, head.length - n)) > 0; ) n += r;
                assertEquals("HTTP/1.1 200", new String(head, 0, n, StandardCharsets.US_ASCII));
            }
            waitFor("the answered POST to be logged as ended", () -> endedNs(carried, 1) != 0);
            Map<String, Object> second = posts(carried).get(1);
            assertEquals(Boolean.FALSE, second.get("held"));
            assertEquals("answered 200", second.get("endedBy"));
            assertTrue("one POST at a time", (Long) second.get("receivedNs") >= (Long) first.get("endedNs"));
            assertEquals(0, openPosts(carried));

            call(carried, "close");
            startedBy = null;
            assertFalse("closing takes it off the board", board.containsKey("Bench: Limelight Reload fake"));
            try (Socket refused = new Socket("127.0.0.1", port)) {
                fail("the port still accepts connections after close (" + refused + ")");
            } catch (ConnectException expected) {
            }
        } finally {
            if (startedBy != null) call(startedBy, "close");
        }
    }

    @Test
    public void startRefusesASecondFakeOnTheBoard() throws Exception {
        Map<String, Object> board = new HashMap<>();
        FakeLimelightServer one = new FakeLimelightServer(0, 0, 8388609.0, () -> 0L);
        FakeLimelightServer two = new FakeLimelightServer(0, 0, 8388609.0, () -> 0L);
        ReloadFake fake = ReloadFake.start(board, one);
        try {
            ReloadFake.start(board, two);
            fail("a second fake replaced the one on the board");
        } catch (IllegalStateException expected) {
            assertEquals(1, board.size());
        } finally {
            fake.close();
            two.close();
        }
        assertTrue(board.isEmpty());
        assertEquals(null, ReloadFake.carried(board));
    }
}
