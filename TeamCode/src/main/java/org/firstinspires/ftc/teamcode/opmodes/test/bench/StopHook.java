package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import org.firstinspires.ftc.teamcode.architecture.core.Module;

final class StopHook extends Module {
    private final Runnable onStop;

    StopHook(Runnable onStop) {
        this.onStop = onStop;
        setTelemetryEnabled(false);
    }

    @Override protected void initStates() {}
    @Override protected void read() {}
    @Override protected void write() {}

    @Override
    public void stop() {
        onStop.run();
    }
}
