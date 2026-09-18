package org.firstinspires.ftc.teamcode.architecture.hardware;

/** Last-written-value cache + clamp + voltage scaling, shared by the Enhanced* wrappers. */
final class WriteCache {
    double tolerance = 0.0;
    double cached = Double.NaN;
    double min = -1.0;
    double max = 1.0;

    boolean voltageCompensationEnabled = false;
    double referenceVoltage = 13.0;

    // A failed Lynx bulk read returns 0 (→ Infinity → full power) or negative (→ reversed); never scale on one.
    private static final double MIN_VALID_VOLTAGE = 6.0;

    boolean shouldWrite(double newValue) {
        return Math.abs(newValue - cached) > tolerance
                || (newValue == 0.0 && cached != 0.0)
                || (newValue != 0.0 && cached == 0.0)
                || (newValue >= max && !(cached >= max))
                || (newValue <= min && !(cached <= min))
                || Double.isNaN(cached);
    }

    double clamp(double value) {
        return Math.max(min, Math.min(max, value));
    }

    double applyVoltageScaling(double power) {
        if (!voltageCompensationEnabled || power == 0.0) return power;
        double measured = BatteryVoltage.get();
        if (measured < MIN_VALID_VOLTAGE) return power;
        return power * (referenceVoltage / measured);
    }

    void store(double value) {
        cached = value;
    }
}
