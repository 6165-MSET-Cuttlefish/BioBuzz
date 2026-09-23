package org.firstinspires.ftc.teamcode.architecture.input;

import java.util.function.DoubleSupplier;

public class CachedDoubleSupplier {
    private final DoubleSupplier doubleSupplier;
    private long lastUpdatedFrame = -1L;
    private double current;

    public CachedDoubleSupplier(DoubleSupplier doubleSupplier) {
        this.doubleSupplier = doubleSupplier;
        this.current = doubleSupplier.getAsDouble();
    }

    public void invalidate() {
        lastUpdatedFrame = -1L;
    }

    public void primeToCurrentState() {
        current = doubleSupplier.getAsDouble();
        lastUpdatedFrame = InputClock.current();
    }

    public double getValue() {
        long frame = InputClock.current();
        if (lastUpdatedFrame != frame) {
            current = doubleSupplier.getAsDouble();
            lastUpdatedFrame = frame;
        }
        return current;
    }

    public EdgeBooleanSupplier greaterThan(double threshold) {
        return new EdgeBooleanSupplier(() -> this.getValue() > threshold);
    }

    public EdgeBooleanSupplier lessThan(double threshold) {
        return new EdgeBooleanSupplier(() -> this.getValue() < threshold);
    }

    public EdgeBooleanSupplier greaterThanOrEqual(double threshold) {
        return new EdgeBooleanSupplier(() -> this.getValue() >= threshold);
    }

    public EdgeBooleanSupplier lessThanOrEqual(double threshold) {
        return new EdgeBooleanSupplier(() -> this.getValue() <= threshold);
    }

    public EdgeBooleanSupplier inRange(double min, double max) {
        return new EdgeBooleanSupplier(() -> {
            double value = this.getValue();
            return value >= min && value <= max;
        });
    }
}
