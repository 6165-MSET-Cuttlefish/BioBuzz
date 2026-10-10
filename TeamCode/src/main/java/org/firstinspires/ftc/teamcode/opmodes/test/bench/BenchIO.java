package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import android.os.Build;
import android.os.Debug;
import android.system.Os;
import android.system.OsConstants;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Result files, runner parameters, and process/GC counters for the bench OpModes. */
final class BenchIO {
    private BenchIO() {}

    static final String TAG = "BENCH";
    private static final File SHARED_DIR = new File(AppUtil.FIRST_FOLDER, "bench");

    static final String RUNNER_HINT = "run it through scripts/bench/run_bench.py, which writes and activates the bench hardware config";

    /** scripts/bench/run_bench.py reads and writes this folder: the app's own files/bench when it made one with run-as, else /sdcard/FIRST/bench. */
    static File dir() {
        File own = new File(AppUtil.getDefContext().getFilesDir(), "bench");
        return own.isDirectory() ? own : SHARED_DIR;
    }

    /** The runner writes params.json before each OpMode it starts; run by hand there is none and every default applies. */
    static Map<String, Object> params() {
        File params = new File(dir(), "params.json");
        if (!params.exists()) return new LinkedHashMap<>();
        return BenchJson.parseObject(readFile(params));
    }

    static double param(Map<String, Object> params, String key, double def) {
        Object v = params.get(key);
        if (v == null) return def;
        if (!(v instanceof Number)) throw new IllegalArgumentException("bench param " + key + " must be a number, was " + v);
        return ((Number) v).doubleValue();
    }

    static String param(Map<String, Object> params, String key, String def) {
        Object v = params.get(key);
        return v == null ? def : v.toString();
    }

    static void write(String name, Object json) {
        File dir = dir();
        if (!dir.isDirectory() && !dir.mkdirs()) throw new IllegalStateException("can't create " + dir);
        File tmp = new File(dir, name + ".json.tmp");
        File out = new File(dir, name + ".json");
        try (Writer w = new OutputStreamWriter(new FileOutputStream(tmp), StandardCharsets.UTF_8)) {
            w.write(BenchJson.write(json));
        } catch (IOException e) {
            throw new IllegalStateException("can't write " + tmp, e);
        }
        if (!tmp.renameTo(out)) throw new IllegalStateException("can't rename " + tmp + " to " + out);
    }

    static String readFile(File f) {
        StringBuilder sb = new StringBuilder();
        try (BufferedReader r = new BufferedReader(new InputStreamReader(new FileInputStream(f), StandardCharsets.UTF_8))) {
            String line;
            while ((line = r.readLine()) != null) sb.append(line).append('\n');
        } catch (IOException e) {
            throw new IllegalStateException("can't read " + f, e);
        }
        return sb.toString();
    }

    static void log(String format, Object... args) {
        RobotLog.ii(TAG, format, args);
    }

    /** hardwareMap.get with a message that says how to get the device, instead of just that it is missing. */
    static <T> T require(HardwareMap hardwareMap, Class<T> type, String name, String howToGetIt) {
        try {
            return hardwareMap.get(type, name);
        } catch (IllegalArgumentException e) {
            throw new IllegalStateException("No " + type.getSimpleName() + " \"" + name + "\" in the active hardware config: "
                    + howToGetIt, e);
        }
    }

    static Map<String, Object> deviceInfo() {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("model", Build.MODEL);
        m.put("sdkInt", Build.VERSION.SDK_INT);
        m.put("release", Build.VERSION.RELEASE);
        m.put("cpus", Runtime.getRuntime().availableProcessors());
        m.put("maxHeapMb", Runtime.getRuntime().maxMemory() / 1e6);
        m.put("pid", android.os.Process.myPid());
        m.put("wallMs", System.currentTimeMillis());
        return m;
    }

    static List<String> deviceNames(HardwareMap hardwareMap) {
        List<String> names = new ArrayList<>();
        for (HardwareDevice d : hardwareMap) {
            for (String n : hardwareMap.getNamesOf(d)) names.add(n + ":" + d.getClass().getSimpleName());
        }
        Collections.sort(names);
        return names;
    }

    /** ART's runtime counters; bytes-allocated is process-wide, so dashboard and camera threads count too. */
    static final class Gc {
        final long count, timeMs, blockingCount, blockingTimeMs, bytesAllocated, bytesFreed;
        final long wallNanos = System.nanoTime();

        private Gc() {
            count = stat("art.gc.gc-count");
            timeMs = stat("art.gc.gc-time");
            blockingCount = stat("art.gc.blocking-gc-count");
            blockingTimeMs = stat("art.gc.blocking-gc-time");
            bytesAllocated = stat("art.gc.bytes-allocated");
            bytesFreed = stat("art.gc.bytes-freed");
        }

        static Gc now() {
            return new Gc();
        }

        Map<String, Object> since(Gc before, long loops) {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("gcCount", count - before.count);
            m.put("gcTimeMs", timeMs - before.timeMs);
            m.put("blockingGcCount", blockingCount - before.blockingCount);
            m.put("blockingGcTimeMs", blockingTimeMs - before.blockingTimeMs);
            long allocated = bytesAllocated - before.bytesAllocated;
            m.put("bytesAllocated", allocated);
            m.put("seconds", (wallNanos - before.wallNanos) / 1e9);
            if (loops > 0) m.put("bytesAllocatedPerLoop", (double) allocated / loops);
            return m;
        }

        private static long stat(String key) {
            String v = Debug.getRuntimeStat(key);
            if (v == null) return -1;
            return Long.parseLong(v.trim());
        }
    }

    /** CPU ticks of this process, of the whole system, and of every thread here, from /proc. */
    static final class Cpu {
        private static final long CLK_TCK = Os.sysconf(OsConstants._SC_CLK_TCK);

        final long processTicks;
        final long systemBusyTicks;
        final long systemTotalTicks;
        final Map<Integer, String> threadNames = new HashMap<>();
        final Map<Integer, Long> threadTicks = new HashMap<>();
        final long wallNanos = System.nanoTime();

        private Cpu() {
            processTicks = ticksOf(readFile(new File("/proc/self/stat")));
            long total = 0;
            long idle = 0;
            // Newer Android hides /proc/stat from apps; that loses only the whole-system figure, reported as unreadable.
            String stat = readFileOrNull(new File("/proc/stat"));
            if (stat != null) {
                String[] cpu = stat.trim().split("\\s+");
                for (int i = 1; i < cpu.length; i++) {
                    long t = Long.parseLong(cpu[i]);
                    total += t;
                    if (i == 4 || i == 5) idle += t;
                }
            }
            systemTotalTicks = stat == null ? -1 : total;
            systemBusyTicks = total - idle;
            File[] tasks = new File("/proc/self/task").listFiles();
            if (tasks == null) throw new IllegalStateException("can't list /proc/self/task");
            for (File t : tasks) {
                String s = readFileOrNull(new File(t, "stat"));
                if (s == null) continue;
                int tid = Integer.parseInt(t.getName());
                threadNames.put(tid, s.substring(s.indexOf('(') + 1, s.lastIndexOf(')')));
                threadTicks.put(tid, ticksOf(s));
            }
        }

        static Cpu now() {
            return new Cpu();
        }

        Map<String, Object> since(Cpu before) {
            Map<String, Object> m = new LinkedHashMap<>();
            double seconds = (wallNanos - before.wallNanos) / 1e9;
            m.put("seconds", seconds);
            m.put("processCpuPercentOfOneCore", 100.0 * (processTicks - before.processTicks) / CLK_TCK / seconds);
            long total = systemTotalTicks - before.systemTotalTicks;
            if (systemTotalTicks < 0 || before.systemTotalTicks < 0) m.put("systemBusyPercentAllCores", "unreadable: /proc/stat");
            else m.put("systemBusyPercentAllCores", total == 0 ? 0 : 100.0 * (systemBusyTicks - before.systemBusyTicks) / total);
            List<Map.Entry<Integer, Long>> deltas = new ArrayList<>();
            for (Map.Entry<Integer, Long> e : threadTicks.entrySet()) {
                Long old = before.threadTicks.get(e.getKey());
                long d = e.getValue() - (old == null ? 0 : old);
                if (d > 0) deltas.add(new java.util.AbstractMap.SimpleEntry<>(e.getKey(), d));
            }
            Collections.sort(deltas, (a, b) -> Long.compare(b.getValue(), a.getValue()));
            List<Map<String, Object>> top = new ArrayList<>();
            for (int i = 0; i < Math.min(12, deltas.size()); i++) {
                Map<String, Object> t = new LinkedHashMap<>();
                t.put("thread", threadNames.get(deltas.get(i).getKey()));
                t.put("cpuPercentOfOneCore", 100.0 * deltas.get(i).getValue() / CLK_TCK / seconds);
                top.add(t);
            }
            m.put("busiestThreads", top);
            return m;
        }

        private static long ticksOf(String stat) {
            String[] f = stat.substring(stat.lastIndexOf(')') + 2).trim().split("\\s+");
            // After the comm field: state is f[0], utime f[11], stime f[12].
            return Long.parseLong(f[11]) + Long.parseLong(f[12]);
        }

        /** The first line, or null for a thread that exited between listing and reading, or an unreadable file. */
        private static String readFileOrNull(File f) {
            try (BufferedReader r = new BufferedReader(new InputStreamReader(new FileInputStream(f), StandardCharsets.UTF_8))) {
                return r.readLine();
            } catch (IOException e) {
                return null;
            }
        }
    }
}
