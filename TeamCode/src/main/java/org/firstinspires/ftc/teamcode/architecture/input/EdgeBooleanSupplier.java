package org.firstinspires.ftc.teamcode.architecture.input;

import java.util.function.BooleanSupplier;

public class EdgeBooleanSupplier {
    private final BooleanSupplier booleanSupplier;

    private boolean previous;
    private boolean current;
    // Last InputClock frame refreshed on; -1 forces a refresh on first query.
    private long lastUpdatedFrame = -1L;

    public EdgeBooleanSupplier(BooleanSupplier booleanSupplier) {
        this.booleanSupplier = booleanSupplier;
        this.previous = booleanSupplier.getAsBoolean();
        this.current = previous;
    }

    public void invalidate() {
        // Not an unconditional refresh: a second one in the same loop would erase the edge just detected.
        ensureFresh();
    }

    public void primeToCurrentState() {
        boolean state = booleanSupplier.getAsBoolean();
        previous = state;
        current = state;
        lastUpdatedFrame = InputClock.current();
    }

    private void ensureFresh() {
        long frame = InputClock.current();
        if (lastUpdatedFrame != frame) {
            previous = current;
            current = booleanSupplier.getAsBoolean();
            lastUpdatedFrame = frame;
        }
    }

    public boolean getValue() {
        ensureFresh();
        return current;
    }

    public boolean wasJustPressed() {
        ensureFresh();
        return current && !previous;
    }

    public boolean wasJustReleased() {
        ensureFresh();
        return !current && previous;
    }
}
