package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.math.Pose;

/** Fixed-capacity ring of recent robot positions; the oldest is dropped once it is full. */
public final class PoseRing {
    private final double[] x;
    private final double[] y;
    private final int capacity;
    private int head;
    private int size;
    private long lastRecordMs = Long.MIN_VALUE;

    public PoseRing(int capacity) {
        if (capacity < 1) throw new IllegalArgumentException("capacity must be >= 1 but was " + capacity);
        this.capacity = capacity;
        x = new double[capacity];
        y = new double[capacity];
    }

    public void record(Pose p) {
        // Time-gated, not per-loop: 30 samples at 50 ms is the 1.5 s trail Pedro 2.x drew, independent of loop rate.
        long now = System.currentTimeMillis();
        if (now - lastRecordMs < 50) return;
        lastRecordMs = now;
        x[head] = p.x();
        y[head] = p.y();
        head = (head + 1) % capacity;
        if (size < capacity) size++;
    }

    public void clear() {
        head = 0;
        size = 0;
        lastRecordMs = Long.MIN_VALUE;
    }

    public int size() {
        return size;
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
