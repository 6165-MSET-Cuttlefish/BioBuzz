package org.firstinspires.ftc.teamcode.pedro;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.architecture.hardware.GuardedOctoQuadLocalizer;
import org.firstinspires.ftc.teamcode.biobuzz.modules.Drivetrain;

@Config("BioBuzz Constants")
public class BioBuzzConstants {

    // Not public, so the dashboard (which lists public fields only) can't open the gate: it takes a source edit and a deploy.
    static boolean measured = false;

    public static class MecanumSettings {
        public DcMotorSimple.Direction frontLeftDirection = DcMotorSimple.Direction.REVERSE;
        public DcMotorSimple.Direction frontRightDirection = DcMotorSimple.Direction.FORWARD;
        public DcMotorSimple.Direction backLeftDirection = DcMotorSimple.Direction.REVERSE;
        public DcMotorSimple.Direction backRightDirection = DcMotorSimple.Direction.FORWARD;
    }

    public static class OctoQuadSettings {
        public String name = "octoquad";
        public int forwardPodChannel = 0;
        public int strafePodChannel = 1;
        public OctoQuad.EncoderDirection forwardPodDirection = OctoQuad.EncoderDirection.REVERSE;
        public OctoQuad.EncoderDirection strafePodDirection = OctoQuad.EncoderDirection.REVERSE;
        public double countsPerMm = 19.89436789;
        public double forwardPodLeftOfCentreIn = -6.1875;
        public double strafePodForwardOfCentreIn = 1.34375;
        public double headingScalar = 1.0085;
        public int velocityIntervalMs = 25;
    }

    public static class ForesightSettings {
        public double primaryForwardP = 0.5101982626923285;
        public double secondaryForwardP = 0.18850462531615828;
        public double primaryLateralP = 1.1272740084866855;
        public double secondaryLateralP = 0.4164976248195712;
        public double translationalSwitchInches = 2.5;

        public double coastFF = 0.013988930617921795;
        public double brakeFF = 0.011890591025233526;

        public double headingP = 5.618075516287591;
        public double headingBrakeLinear = 0.04586686604931309;
        public double headingBrakeQuadratic = 0.00885564987142792;

        public double linearBrakeForward = 0.11619749704491415;
        public double linearBrakeStrafe = 0.03999416772644396;
        public double quadraticBrakeForward = 0.001387133564003021;
        public double quadraticBrakeStrafe = 0.0027934483235117686;

        public double maxAchievableForwardVelocity = 73.00734154443577;
        public double maxAchievableStrafeVelocity = 60.048785872434564;
        public double naturalForwardDeceleration = 61.973506354027336;
        public double naturalStrafeDeceleration = 78.27862874148664;
    }

    public static MecanumSettings mecanum = new MecanumSettings();
    public static OctoQuadSettings octoQuad = new OctoQuadSettings();
    public static ForesightSettings foresight = new ForesightSettings();

    public static MecanumConfig drivetrainConfig() {
        return new MecanumConfig(c -> {
            c.frontLeftName.set(Drivetrain.FRONT_LEFT);
            c.frontRightName.set(Drivetrain.FRONT_RIGHT);
            c.backLeftName.set(Drivetrain.BACK_LEFT);
            c.backRightName.set(Drivetrain.BACK_RIGHT);
            c.frontLeftDirection.set(mecanum.frontLeftDirection);
            c.frontRightDirection.set(mecanum.frontRightDirection);
            c.backLeftDirection.set(mecanum.backLeftDirection);
            c.backRightDirection.set(mecanum.backRightDirection);
        });
    }

    public static OctoQuadConfig localizerConfig() {
        return new OctoQuadConfig(c -> {
            c.name.set(octoQuad.name);
            c.xPodPort.set(octoQuad.forwardPodChannel);
            c.yPodPort.set(octoQuad.strafePodChannel);
            c.xPodDirection.set(octoQuad.forwardPodDirection);
            c.yPodDirection.set(octoQuad.strafePodDirection);
            c.encoderResolutionUnit.set(DistanceUnit.MM);
            c.ticksPerUnit.set(octoQuad.countsPerMm);
            c.offsetUnits.set(DistanceUnit.INCH);
            c.xPodOffset.set(octoQuad.forwardPodLeftOfCentreIn);
            c.yPodOffset.set(octoQuad.strafePodForwardOfCentreIn);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.headingScalar.set(octoQuad.headingScalar);
            c.localizerVelocityIntervalMS.set(octoQuad.velocityIntervalMs);
            c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        });
    }

    // Controller gains are read live through suppliers; every other value is copied in when the follower is built at INIT.
    public static ForesightConfig foresightConfig() {
        return new ForesightConfig(c -> {
            c.forwardTranslational.set(Controller.piecewise(Controller.proportional(() -> foresight.secondaryForwardP))
                    .put(foresight.translationalSwitchInches, Controller.proportional(() -> foresight.primaryForwardP)));
            c.strafeTranslational.set(Controller.piecewise(Controller.proportional(() -> foresight.secondaryLateralP))
                    .put(foresight.translationalSwitchInches, Controller.proportional(() -> foresight.primaryLateralP)));

            c.coast.set(Controller.proportionalFeedforward(() -> foresight.coastFF));
            c.brake.set(Controller.proportionalFeedforward(() -> foresight.brakeFF));

            c.headingFeedback.set(Controller.proportional(() -> foresight.headingP));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(foresight.headingBrakeLinear, foresight.headingBrakeQuadratic));

            c.linearBrakeCoefficients.set(Matrix.diag(foresight.linearBrakeForward, foresight.linearBrakeStrafe));
            c.quadraticBrakeCoefficients.set(Matrix.diag(foresight.quadraticBrakeForward, foresight.quadraticBrakeStrafe));

            c.maxAchievableForwardVelocity.set(foresight.maxAchievableForwardVelocity);
            c.maxAchievableStrafeVelocity.set(foresight.maxAchievableStrafeVelocity);
            c.naturalForwardDeceleration.set(foresight.naturalForwardDeceleration);
            c.naturalStrafeDeceleration.set(foresight.naturalStrafeDeceleration);
        });
    }

    public static GuardedOctoQuadLocalizer localizer(HardwareMap hardwareMap) {
        return new GuardedOctoQuadLocalizer(hardwareMap, localizerConfig());
    }

    /** For code that isn't an EnhancedOpMode, whose START would otherwise do the wait. */
    public static GuardedOctoQuadLocalizer readyLocalizer(HardwareMap hardwareMap) {
        GuardedOctoQuadLocalizer localizer = localizer(hardwareMap);
        localizer.awaitRunning();
        return localizer;
    }

    public static Mecanum drivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig());
    }

    public static Foresight foresight() {
        return new Foresight(foresightConfig());
    }

    /** Throws, before building anything, for an OpMode not marked {@link PedroNeverDrives} until {@link #measured} is set. */
    public static Follower create(OpMode opMode) {
        if (!opMode.getClass().isAnnotationPresent(PedroNeverDrives.class)) requireMeasured(displayName(opMode.getClass()));
        return new Follower(localizer(opMode.hardwareMap), drivetrain(opMode.hardwareMap), foresight());
    }

    public static boolean isMeasured() {
        return measured;
    }

    public static void requireMeasured(String what) {
        if (measured) return;
        throw new IllegalStateException(what + " lets Pedro drive the robot, so first measure the BIOBUZZ robot's drive "
                + "directions, odometry pods, IMU heading scale and Foresight gains by docs/odometry-tuning.md and set "
                + "BioBuzzConstants.measured to true: apart from the pod channels, the values in BioBuzzConstants are still the retired Betta bot's.");
    }

    static String displayName(Class<?> opMode) {
        Autonomous auto = opMode.getAnnotation(Autonomous.class);
        String name = auto != null ? auto.name() : "";
        TeleOp tele = opMode.getAnnotation(TeleOp.class);
        if (name.isEmpty() && tele != null) name = tele.name();
        return name.isEmpty() ? opMode.getSimpleName() : name;
    }
}
