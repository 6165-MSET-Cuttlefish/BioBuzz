package org.firstinspires.ftc.teamcode.architecture.hardware;

/** Shared voltage. EnhancedOpMode publishes once per loop; wrappers pull on demand. */
public final class BatteryVoltage {
    private static volatile double current = 12.0;

    private BatteryVoltage() {}

    public static void update(double voltage) {
        current = voltage;
    }

    public static double get() {
        return current;
    }
}
