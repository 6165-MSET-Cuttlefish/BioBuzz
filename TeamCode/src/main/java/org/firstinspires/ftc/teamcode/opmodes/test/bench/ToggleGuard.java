package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import org.firstinspires.ftc.teamcode.architecture.OptimizationToggles;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.modules.Drivetrain;

import java.util.LinkedHashMap;
import java.util.Map;

/** Restores the toggles from stop(), which runs even when a hook throws; statics would otherwise leak into the next OpMode. */
final class ToggleGuard extends Module {
    private final int dashboardIntervalMs = OptimizationToggles.dashboardTransmissionIntervalMs;
    private final int dsIntervalMs = OptimizationToggles.dsTransmissionIntervalMs;
    private final int currentEveryN = OptimizationToggles.currentReadEveryNLoops;
    private final boolean dsTelemetry = OptimizationToggles.dsTelemetry;
    private final boolean dashboardTelemetry = OptimizationToggles.dashboardTelemetry;
    private final boolean voltageTelemetry = OptimizationToggles.voltageTelemetry;
    private final boolean currentTelemetry = OptimizationToggles.currentTelemetry;
    private final boolean loopProfile = OptimizationToggles.loopProfile;
    private final boolean encoderTelemetry = Drivetrain.encoderTelemetry;
    private final boolean motorCurrentTelemetry = Drivetrain.motorCurrentTelemetry;

    ToggleGuard() {
        setTelemetryEnabled(false);
    }

    void restore() {
        OptimizationToggles.dashboardTransmissionIntervalMs = dashboardIntervalMs;
        OptimizationToggles.dsTransmissionIntervalMs = dsIntervalMs;
        OptimizationToggles.currentReadEveryNLoops = currentEveryN;
        OptimizationToggles.dsTelemetry = dsTelemetry;
        OptimizationToggles.dashboardTelemetry = dashboardTelemetry;
        OptimizationToggles.voltageTelemetry = voltageTelemetry;
        OptimizationToggles.currentTelemetry = currentTelemetry;
        OptimizationToggles.loopProfile = loopProfile;
        Drivetrain.encoderTelemetry = encoderTelemetry;
        Drivetrain.motorCurrentTelemetry = motorCurrentTelemetry;
    }

    static Map<String, Object> current() {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("dashboardTransmissionIntervalMs", OptimizationToggles.dashboardTransmissionIntervalMs);
        m.put("dsTransmissionIntervalMs", OptimizationToggles.dsTransmissionIntervalMs);
        m.put("currentReadEveryNLoops", OptimizationToggles.currentReadEveryNLoops);
        m.put("telemetryLazyFormat", OptimizationToggles.telemetryLazyFormat);
        m.put("dsTelemetry", OptimizationToggles.dsTelemetry);
        m.put("dashboardTelemetry", OptimizationToggles.dashboardTelemetry);
        m.put("voltageTelemetry", OptimizationToggles.voltageTelemetry);
        m.put("currentTelemetry", OptimizationToggles.currentTelemetry);
        m.put("loopProfile", OptimizationToggles.loopProfile);
        m.put("drivetrainEncoderTelemetry", Drivetrain.encoderTelemetry);
        m.put("drivetrainMotorCurrentTelemetry", Drivetrain.motorCurrentTelemetry);
        return m;
    }

    @Override protected void initStates() {}
    @Override protected void read() {}
    @Override protected void write() {}

    @Override
    public void stop() {
        restore();
    }
}
