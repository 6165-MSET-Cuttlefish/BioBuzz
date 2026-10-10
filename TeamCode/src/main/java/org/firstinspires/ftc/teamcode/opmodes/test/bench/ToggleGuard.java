package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import org.firstinspires.ftc.teamcode.architecture.OptimizationToggles;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.modules.Drivetrain;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Snapshots the static telemetry and profiler toggles a bench flips and puts them back from stop(), which the
 * framework's safe-state pass runs even when a hook throws; statics would otherwise leak into the next OpMode.
 */
final class ToggleGuard extends Module {
    private final int dashboardIntervalMs = OptimizationToggles.dashboardTransmissionIntervalMs;
    private final int dsIntervalMs = OptimizationToggles.dsTransmissionIntervalMs;
    private final boolean profilerEnabled = OptimizationToggles.profilerEnabled;
    private final int currentEveryN = OptimizationToggles.currentReadEveryNLoops;
    private final boolean dsTelemetry = Robot.telemetryToggles.dsTelemetry;
    private final boolean dashboardTelemetry = Robot.telemetryToggles.dashboardTelemetry;
    private final boolean voltage = Robot.telemetryToggles.voltage;
    private final boolean current = Robot.telemetryToggles.current;
    private final boolean loopProfile = Robot.telemetryToggles.loopProfile;
    private final boolean encoderTelemetry = Drivetrain.encoderTelemetry;
    private final boolean motorCurrentTelemetry = Drivetrain.motorCurrentTelemetry;

    ToggleGuard() {
        setTelemetryEnabled(false);
    }

    void restore() {
        OptimizationToggles.dashboardTransmissionIntervalMs = dashboardIntervalMs;
        OptimizationToggles.dsTransmissionIntervalMs = dsIntervalMs;
        OptimizationToggles.profilerEnabled = profilerEnabled;
        OptimizationToggles.currentReadEveryNLoops = currentEveryN;
        Robot.telemetryToggles.dsTelemetry = dsTelemetry;
        Robot.telemetryToggles.dashboardTelemetry = dashboardTelemetry;
        Robot.telemetryToggles.voltage = voltage;
        Robot.telemetryToggles.current = current;
        Robot.telemetryToggles.loopProfile = loopProfile;
        Drivetrain.encoderTelemetry = encoderTelemetry;
        Drivetrain.motorCurrentTelemetry = motorCurrentTelemetry;
    }

    static Map<String, Object> current() {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("dashboardTransmissionIntervalMs", OptimizationToggles.dashboardTransmissionIntervalMs);
        m.put("dsTransmissionIntervalMs", OptimizationToggles.dsTransmissionIntervalMs);
        m.put("profilerEnabled", OptimizationToggles.profilerEnabled);
        m.put("currentReadEveryNLoops", OptimizationToggles.currentReadEveryNLoops);
        m.put("telemetryLazyFormat", OptimizationToggles.telemetryLazyFormat);
        m.put("dsTelemetry", Robot.telemetryToggles.dsTelemetry);
        m.put("dashboardTelemetry", Robot.telemetryToggles.dashboardTelemetry);
        m.put("voltage", Robot.telemetryToggles.voltage);
        m.put("current", Robot.telemetryToggles.current);
        m.put("loopProfile", Robot.telemetryToggles.loopProfile);
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
