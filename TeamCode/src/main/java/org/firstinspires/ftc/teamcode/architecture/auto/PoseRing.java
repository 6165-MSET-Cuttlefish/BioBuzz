package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.math.Pose;

/** Fixed-capacity ring of recent robot positions; the oldest is dropped once it is full. */
public final class PoseRing {
    private static final long RECORD_INTERVAL_NS = 50_000_000L;

    private final double[] x;
    private final double[] y;
    private final int capacity;
    private int head;
    private int size;
    private long lastRecordNs;

    public PoseRing(int capacity) {
        if (capacity < 1) throw new IllegalArgumentException("capacity must be >= 1 but was " + capacity);
        this.capacity = capacity;
        x = new double[capacity];
        y = new double[capacity];
    }

    public void record(Pose p) {
        // Time-gated so the trail spans capacity x 50 ms regardless of loop rate.
        long now = System.nanoTime();
        if (size > 0 && now - lastRecordNs < RECORD_INTERVAL_NS) return;
        lastRecordNs = now;
        x[head] = p.x();
        y[head] = p.y();
        head = (head + 1) % capacity;
        if (size < capacity) size++;
    }

    /** Oldest first. */
    public double[] xs() {
        return unrolled(x);
    }

    /** Oldest first. */
    public double[] ys() {
        return unrolled(y);
    }

    private double[] unrolled(double[] src) {
        double[] out = new double[size];
        int start = (head - size + capacity) % capacity;
        for (int i = 0; i < size; i++) out[i] = src[(start + i) % capacity];
        return out;
    }
}
