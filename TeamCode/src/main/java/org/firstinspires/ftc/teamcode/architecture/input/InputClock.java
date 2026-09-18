package org.firstinspires.ftc.teamcode.architecture.input;

/**
 * Monotonic per-loop counter; {@code EnhancedOpMode} calls {@link #advance()} once per tick and edge
 * suppliers key their once-per-loop refresh off it, whether or not a {@link LayeredGamepad} pumps them.
 * Static by design — hot-reload safe, since a monotonic counter has no stale state.
 */
public final class InputClock {
    private static long frame = 0;

    private InputClock() {}

    public static long current() { return frame; }

    public static void advance() { frame++; }
}
