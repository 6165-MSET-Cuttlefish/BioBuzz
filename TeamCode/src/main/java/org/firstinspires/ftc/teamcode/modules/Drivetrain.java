package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.control.PidController;
import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;

@Config
public class Drivetrain extends Module {
    private final EnhancedMotor fl, bl, br, fr;
    /** Whole-drivetrain current sensor on an analog port; null when the robot config has none. */
    private final AnalogInput floodgate;
    private Follower follower;

    public final ElapsedTime bonkTimer = new ElapsedTime();

    private static final double ENCODER_TO_RPM = 60.0 / 537.7;

    public static class EnableMotors {
        public boolean enableFl = true;
        public boolean enableBl = true;
        public boolean enableBr = true;
        public boolean enableFr = true;
    }

    public static class CurrentLimiterConfig {
        public boolean enabled = true;
        public double currentThresholdMin = 25.0;
        public double currentThresholdMax = 30.0;
        public double currentOverTime = 0;
        public double minCurrentOverTime = 2;
        public double integratedCurrentLimit = 400000;
        public double decayRate = 0.9;
        public double decayLoopMs = 5;
    }

    public static class TelemetryToggles {
        public boolean TOGGLE = true;
        public boolean current = false;
    }

    public static EnableMotors enableMotors = new EnableMotors();
    public static CurrentLimiterConfig currentLimiterConfig = new CurrentLimiterConfig();
    public static TelemetryToggles telemetryToggles = new TelemetryToggles();
    private ElapsedTime currentLoopTimer;

    private boolean headingLocked = false;
    private double lockedHeading = 0;
    private final PidController headingLockController = new PidController()
            .withGains(1, 0, 0.2, 0.0)
            .withFeedforward(0, 0)
            .withContinuousInput(-Math.PI, Math.PI);

    public enum DriveState implements State {
        MANUAL(0),
        EXTERNAL(0);

        DriveState(double value) {
            setValue(value);
        }
    }

    private double flPower, blPower, brPower, frPower;
    private double lastCurrentLimiterMultiplier = 1.0;

    public Drivetrain(HardwareMap hardwareMap) {
        super();

        fl = new EnhancedMotor(hardwareMap, "fl").withCachingTolerance(0.05);
        bl = new EnhancedMotor(hardwareMap, "bl").withCachingTolerance(0.05);
        fr = new EnhancedMotor(hardwareMap, "fr").withCachingTolerance(0.05);
        br = new EnhancedMotor(hardwareMap, "br").withCachingTolerance(0.05);

        floodgate = hardwareMap.tryGet(AnalogInput.class, "floodgate");
    }

    /** Required for isBonk(), lockHeading() and field-centric drive; robot-centric drive works without it. */
    public Drivetrain withFollower(Follower follower) {
        this.follower = follower;
        return this;
    }

    private Follower requireFollower() {
        if (follower == null) {
            throw new IllegalStateException("Drivetrain needs a Follower for this call; pass one with withFollower()");
        }
        return follower;
    }

    @Override
    public void init() {
        super.init();
        currentLoopTimer = new ElapsedTime();
    }

    @Override
    protected void initStates() {
        setStates(DriveState.MANUAL);
    }

    @Override
    protected void read() {
    }

    @Override
    protected void write() {
        if (!getState(DriveState.class).equals(DriveState.EXTERNAL)) {
            fl.setPower(enableMotors.enableFl ? flPower : 0);
            bl.setPower(enableMotors.enableBl ? blPower : 0);
            br.setPower(enableMotors.enableBr ? brPower : 0);
            fr.setPower(enableMotors.enableFr ? frPower : 0);
        }
    }

    public void setTargets(double fl, double bl, double br, double fr) {
        double maxPower = Math.max(
                Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br))));

        if (maxPower > 1.0) {
            fl /= maxPower;
            bl /= maxPower;
            fr /= maxPower;
            br /= maxPower;
        }

        // The floodgate is an analog input (served by the once-per-loop bulk read, no extra bus call),
        // so compute every loop — finer sampling for the I²·t integral, at no cost.
        lastCurrentLimiterMultiplier = computeCurrentLimiterMultiplier();
        currentLoopTimer.reset();
        flPower = fl * lastCurrentLimiterMultiplier;
        blPower = bl * lastCurrentLimiterMultiplier;
        brPower = br * lastCurrentLimiterMultiplier;
        frPower = fr * lastCurrentLimiterMultiplier;
    }

    public void setRawTargets(double fl, double bl, double br, double fr) {
        flPower = fl;
        blPower = bl;
        brPower = br;
        frPower = fr;
    }

    @Override
    public void stop() {
        setRawTargets(0, 0, 0, 0);
        // write() never runs after EnhancedOpMode.stop(), so the cut must go to the motors here.
        fl.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        fr.setPower(0);
    }

    public boolean isBonk() {
        Follower f = requireFollower();
        if (f.isBusy() && !f.atParametricEnd() && f.velocity().toVector2D().magnitude() < 2) {
            if (bonkTimer.milliseconds() > 2000) {
                return true;
            }
        } else {
            bonkTimer.reset();
        }
        return false;
    }

    public boolean hasFloodgate() {
        return floodgate != null;
    }

    public double getFloodgateCurrent() {
        if (floodgate == null) return 0.0;
        double voltage = floodgate.getVoltage();
        return (voltage / 3.3) * 80.0;
    }

    private double computeCurrentLimiterMultiplier() {
        if (floodgate == null || !currentLimiterConfig.enabled) return 1.0;

        double current = getFloodgateCurrent();
        currentLimiterConfig.currentOverTime += Math.pow(current, 2) * currentLoopTimer.milliseconds();
        currentLimiterConfig.currentOverTime *= Math.pow(currentLimiterConfig.decayRate, (currentLoopTimer.milliseconds() / currentLimiterConfig.decayLoopMs));

        if (current <= currentLimiterConfig.currentThresholdMin) {
            return 1.0;
        }

        // Integral (I²·t) scaling rather than a hard cutoff; integratedCurrentLimit is the ceiling I²·t shouldn't exceed.
        return 1.0 - Range.clip(currentLimiterConfig.currentOverTime / currentLimiterConfig.integratedCurrentLimit, 0, 1);
    }

    public void lockHeading() {
        lockedHeading = requireFollower().pose().heading();
        headingLockController.reset();
        headingLocked = true;
    }

    public void lockHeading(double headingRadians) {
        requireFollower();
        lockedHeading = headingRadians;
        headingLockController.reset();
        headingLocked = true;
    }

    public void unlockHeading() {
        headingLocked = false;
    }

    public boolean isHeadingLocked() {
        return headingLocked;
    }

    public void setMecanumTargets(double y, double x, double rx, boolean fieldCentric) {
        if (fieldCentric) {
            double heading = -requireFollower().pose().heading();
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);

            if (Context.allianceColor == AllianceColor.BLUE) {
                y = -y;
            }

            double rotatedX = x * cos - y * sin;
            double rotatedY = x * sin + y * cos;

            y = rotatedY;
            x = rotatedX;
        }

        if (headingLocked) {
            double currentHeading = requireFollower().pose().heading();
            double headingError = Angle.error(currentHeading, lockedHeading);
            headingLockController.update(headingError, 0);
            rx = -Range.clip(headingLockController.calculate(), -1, 1);
        }

        double frontLeft = y + x + rx;
        double backLeft = y - x + rx;
        double frontRight = y - x - rx;
        double backRight = y + x - rx;

        setTargets(frontLeft, backLeft, backRight, frontRight);
    }

    public EnhancedMotor getFl() {
        return fl;
    }

    public EnhancedMotor getBl() {
        return bl;
    }

    public EnhancedMotor getBr() {
        return br;
    }

    public EnhancedMotor getFr() {
        return fr;
    }

    public double[] getMotorPowers() {
        return new double[]{flPower, blPower, brPower, frPower};
    }

    @Override
    protected void onTelemetry() {
        if (!telemetryToggles.TOGGLE) return;

        logDashboard("Drive Mode", getState(DriveState.class));
        logDashboard("Motor Powers", "FL:%.2f BL:%.2f FR:%.2f BR:%.2f", flPower, blPower, frPower, brPower);

        logDashboard("FL Position (ticks)", fl.getCurrentPosition());
        logDashboard("FR Position (ticks)", fr.getCurrentPosition());
        logDashboard("FL Velocity (RPM)", "%.1f", fl.getVelocity() * ENCODER_TO_RPM);
        logDashboard("BL Velocity (RPM)", "%.1f", bl.getVelocity() * ENCODER_TO_RPM);
        logDashboard("BR Velocity (RPM)", "%.1f", br.getVelocity() * ENCODER_TO_RPM);
        logDashboard("FR Velocity (RPM)", "%.1f", fr.getVelocity() * ENCODER_TO_RPM);

        if (floodgate != null) {
            logDashboard("Floodgate Current (A)", "%.2f", getFloodgateCurrent());
            logDashboard("Current Limiter Multiplier", "%.2f", lastCurrentLimiterMultiplier);
            logDashboard("currentOverTime", "%.2f", currentLimiterConfig.currentOverTime);
            logDashboard("final current scale", "%.3f", 1.0 - currentLimiterConfig.currentOverTime / currentLimiterConfig.integratedCurrentLimit);
        }

        if (telemetryToggles.current) {
            logDashboard("FL Current (A)", "%.2f", fl.getCurrent(CurrentUnit.AMPS));
            logDashboard("BL Current (A)", "%.2f", bl.getCurrent(CurrentUnit.AMPS));
            logDashboard("BR Current (A)", "%.2f", br.getCurrent(CurrentUnit.AMPS));
            logDashboard("FR Current (A)", "%.2f", fr.getCurrent(CurrentUnit.AMPS));
        }
    }
}
