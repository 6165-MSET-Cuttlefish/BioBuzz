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

    public ConditionalBinder conditionalBindState() {
        return new ConditionalBinder(this);
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

    public static class ConditionalBinder {
        private final CachedDoubleSupplier supplier;
        private double minValue = Double.NEGATIVE_INFINITY;
        private double maxValue = Double.POSITIVE_INFINITY;
        private boolean minInclusive = false;
        private boolean maxInclusive = false;

        public ConditionalBinder(CachedDoubleSupplier supplier) {
            this.supplier = supplier;
        }

        public ConditionalBinder greaterThan(double value) {
            this.minValue = value;
            this.minInclusive = false;
            return this;
        }

        public ConditionalBinder greaterThanEqualTo(double value) {
            this.minValue = value;
            this.minInclusive = true;
            return this;
        }

        public ConditionalBinder lessThan(double value) {
            this.maxValue = value;
            this.maxInclusive = false;
            return this;
        }

        public ConditionalBinder lessThanEqualTo(double value) {
            this.maxValue = value;
            this.maxInclusive = true;
            return this;
        }

        public EdgeBooleanSupplier bind() {
            return new EdgeBooleanSupplier(() -> {
                double value = supplier.getValue();
                boolean minCheck = minInclusive ? value >= minValue : value > minValue;
                boolean maxCheck = maxInclusive ? value <= maxValue : value < maxValue;
                return minCheck && maxCheck;
            });
        }
    }
}
