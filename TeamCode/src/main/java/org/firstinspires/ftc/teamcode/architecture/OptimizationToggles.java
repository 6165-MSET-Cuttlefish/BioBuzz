package org.firstinspires.ftc.teamcode.architecture;

import com.acmerobotics.dashboard.config.Config;

@Config("Optimization Toggles")
public final class OptimizationToggles {
    private OptimizationToggles() {}

    /** Milliseconds slothboard's sender sleeps between batches of queued packets (its own default is 100). */
    public static int dashboardTransmissionIntervalMs = 20;

    /** Minimum milliseconds between Driver Station telemetry sends (the SDK's own default is 250); DS lines are built only when one is due. */
    public static int dsTransmissionIntervalMs = 250;

    public static boolean dashboardSkipFieldImage = false;
    public static boolean dashboardSkipGrid = false;
    public static boolean dashboardSkipPoseHistory = false;

    /** Skip format + Item allocation on telemetry calls outside a DS frame while dashboard telemetry is off. */
    public static boolean telemetryLazyFormat = true;

    public static boolean dsTelemetry = true;
    public static boolean dashboardTelemetry = true;
    public static boolean voltageTelemetry = true;
    public static boolean currentTelemetry = false;
    public static boolean loopProfile = true;

    /** Loops between per-hub Lynx current reads (a bus command, not bulk-cached). */
    public static int currentReadEveryNLoops = 1;
}
