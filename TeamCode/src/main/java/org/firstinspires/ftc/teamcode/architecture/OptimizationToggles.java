package org.firstinspires.ftc.teamcode.architecture;

import com.acmerobotics.dashboard.config.Config;

/** Live-tunable perf knobs on FTC Dashboard. Defaults favor visibility; tighten for competition. */
@Config
public final class OptimizationToggles {
    private OptimizationToggles() {}

    public static int dashboardEveryNLoops = 1;

    public static boolean dashboardSkipFieldImage = false;
    public static boolean dashboardSkipGrid = false;
    public static boolean dashboardSkipPoseHistory = false;


    public static int telemetryEveryNLoops = 1;

    /** Skip format + Item allocation on telemetry calls when both backends are off. */
    public static boolean telemetryLazyFormat = true;

    /** Sort modules once at init. Only safe when Module.telemetryOrder() values don't change at runtime. */
    public static boolean telemetrySortModulesOnce = true;

    public static boolean profilerEnabled = true;

    /** loopProfile default, read at class load — not live like the rest of this class. */
    public static boolean loopProfileTelemetryByDefault = true;

    /** Lynx whole-hub current read (a real ADC bus command, not bulk-cached); cadence when the
     *  current telemetry toggle is on. The drivetrain floodgate is a bulk-cached analog input, so it
     *  is NOT throttled — only this per-hub ADC read is. */
    public static int currentReadEveryNLoops = 1;
}
