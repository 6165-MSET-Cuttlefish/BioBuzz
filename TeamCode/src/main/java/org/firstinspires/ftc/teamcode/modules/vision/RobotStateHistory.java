package org.firstinspires.ftc.teamcode.modules.vision;

/** Not thread-safe: record and read from the OpMode thread only. */
public final class RobotStateHistory {

    private static final int CAPACITY = 64;

    public static final class Sample {
        public final double x, y, heading;
        public final double vx, vy, omega;
        public final double timestampSeconds;

        public Sample(double x, double y, double heading,
                      double vx, double vy, double omega, double timestampSeconds) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.vx = vx;
            this.vy = vy;
            this.omega = omega;
            this.timestampSeconds = timestampSeconds;
        }
    }

    private final Sample[] samples = new Sample[CAPACITY];
    private int nextIndex = 0;
    private int size = 0;

    public void record(Sample sample) {
        samples[nextIndex] = sample;
        nextIndex = (nextIndex + 1) % CAPACITY;
        if (size < CAPACITY) size++;
    }

    public void clear() {
        nextIndex = 0;
        size = 0;
    }

    public boolean isEmpty() { return size == 0; }

    public Sample newest() {
        return size == 0 ? null : samples[(nextIndex - 1 + CAPACITY) % CAPACITY];
    }

    /** Interpolated; clamps to the oldest/newest sample outside the retained window. */
    public Sample sampleAt(double timestampSeconds) {
        if (size == 0) return null;

        Sample newer = null;
        Sample older = null;
        for (int i = 0; i < size; i++) {
            Sample s = samples[(nextIndex - 1 - i + 2 * CAPACITY) % CAPACITY];
            if (s.timestampSeconds >= timestampSeconds) {
                newer = s;
            } else {
                older = s;
                break;
            }
        }

        if (older == null) return newer;
        if (newer == null) return older;

        double span = newer.timestampSeconds - older.timestampSeconds;
        if (span <= 0) return newer;
        double t = (timestampSeconds - older.timestampSeconds) / span;

        return new Sample(
                lerp(older.x, newer.x, t),
                lerp(older.y, newer.y, t),
                older.heading + normalizeRadians(newer.heading - older.heading) * t,
                lerp(older.vx, newer.vx, t),
                lerp(older.vy, newer.vy, t),
                lerp(older.omega, newer.omega, t),
                timestampSeconds);
    }

    private static double lerp(double a, double b, double t) { return a + (b - a) * t; }

    private static double normalizeRadians(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}
