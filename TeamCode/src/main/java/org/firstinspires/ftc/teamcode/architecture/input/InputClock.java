package org.firstinspires.ftc.teamcode.architecture.input;

/** Static but hot-reload safe: a monotonic counter holds no stale state. */
public final class InputClock {
    private static long frame = 0;

    private InputClock() {}

    public static long current() { return frame; }

    public static void advance() { frame++; }
}
