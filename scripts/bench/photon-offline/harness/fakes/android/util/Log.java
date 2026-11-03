package android.util;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Consumer;

public final class Log {
    public static final int VERBOSE = 2;
    public static final int DEBUG = 3;
    public static final int INFO = 4;
    public static final int WARN = 5;
    public static final int ERROR = 6;
    public static final int ASSERT = 7;
    public static final List<String> LINES = new CopyOnWriteArrayList<>();
    public static volatile Consumer<String> hook;

    private Log() {}

    public static int println(int priority, String tag, String msg) {
        String line = "VDIWEA".charAt(Math.max(0, Math.min(5, priority - 2))) + "/" + tag + ": " + msg;
        LINES.add(line);
        Consumer<String> h = hook;
        if (h != null) h.accept(line);
        if (System.getenv("PHOTON_LOG") != null || priority >= ERROR) System.out.println("    log " + line.split("\n")[0]);
        return 0;
    }

    public static int v(String tag, String msg) { return println(VERBOSE, tag, msg); }
    public static int d(String tag, String msg) { return println(DEBUG, tag, msg); }
    public static int i(String tag, String msg) { return println(INFO, tag, msg); }
    public static int w(String tag, String msg) { return println(WARN, tag, msg); }
    public static int e(String tag, String msg) { return println(ERROR, tag, msg); }
    public static int v(String tag, String msg, Throwable t) { return println(VERBOSE, tag, msg + "\n" + getStackTraceString(t)); }
    public static int d(String tag, String msg, Throwable t) { return println(DEBUG, tag, msg + "\n" + getStackTraceString(t)); }
    public static int i(String tag, String msg, Throwable t) { return println(INFO, tag, msg + "\n" + getStackTraceString(t)); }
    public static int w(String tag, String msg, Throwable t) { return println(WARN, tag, msg + "\n" + getStackTraceString(t)); }
    public static int e(String tag, String msg, Throwable t) { return println(ERROR, tag, msg + "\n" + getStackTraceString(t)); }
    public static boolean isLoggable(String tag, int level) { return true; }

    public static String getStackTraceString(Throwable t) {
        if (t == null) return "";
        StringWriter s = new StringWriter();
        t.printStackTrace(new PrintWriter(s));
        return s.toString();
    }
}
