package org.firstinspires.ftc.teamcode.architecture.core;

import com.qualcomm.robotcore.util.RobotLog;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Named, recoverable faults that {@link EnhancedOpMode} shows first on every telemetry frame. Keys are global;
 * cleared at every {@link EnhancedOpMode} INIT. Thread-safe.
 */
public final class Faults {
    private Faults() {}

    public static final class Fault {
        public final String key;
        public final String message;

        Fault(String key, String message) {
            this.key = key;
            this.message = message;
        }

        @Override public String toString() { return key + ": " + message; }
    }

    private static final String LOG_TAG = "Fault";
    private static final Map<String, Fault> ACTIVE = new LinkedHashMap<>();
    // Copy-on-write so the per-loop render path reads it without locking or allocating.
    private static volatile List<Fault> snapshot = Collections.emptyList();

    /** Raises {@code key}, or updates its message; order of first raise is kept. */
    public static void raise(String key, String message) {
        requireKey(key);
        Objects.requireNonNull(message, "fault message");
        synchronized (ACTIVE) {
            Fault old = ACTIVE.get(key);
            if (old != null && old.message.equals(message)) return;
            ACTIVE.put(key, new Fault(key, message));
            publish();
            if (old == null) RobotLog.ee(LOG_TAG, "%s: %s", key, message);
        }
    }

    public static void clear(String key) {
        requireKey(key);
        synchronized (ACTIVE) {
            if (ACTIVE.remove(key) == null) return;
            publish();
            RobotLog.ii(LOG_TAG, "%s cleared", key);
        }
    }

    public static boolean has(String key) {
        requireKey(key);
        synchronized (ACTIVE) {
            return ACTIVE.containsKey(key);
        }
    }

    public static boolean any() {
        return !snapshot.isEmpty();
    }

    /** Immutable, in order of first raise. */
    public static List<Fault> active() {
        return snapshot;
    }

    static void reset() {
        synchronized (ACTIVE) {
            ACTIVE.clear();
            publish();
        }
    }

    private static void publish() {
        snapshot = ACTIVE.isEmpty()
                ? Collections.<Fault>emptyList()
                : Collections.unmodifiableList(new ArrayList<>(ACTIVE.values()));
    }

    private static void requireKey(String key) {
        if (key == null || key.isEmpty()) throw new IllegalArgumentException("fault key must be non-empty");
    }
}
