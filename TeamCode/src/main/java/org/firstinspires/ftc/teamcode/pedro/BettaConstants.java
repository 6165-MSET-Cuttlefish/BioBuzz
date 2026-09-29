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
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class BettaConstants {

    public static class MecanumSettings {
        public String frontLeftName = "fl";
        public String frontRightName = "fr";
        public String backLeftName = "bl";
        public String backRightName = "br";
        public DcMotorSimple.Direction frontLeftDirection = DcMotorSimple.Direction.REVERSE;
        public DcMotorSimple.Direction frontRightDirection = DcMotorSimple.Direction.FORWARD;
        public DcMotorSimple.Direction backLeftDirection = DcMotorSimple.Direction.REVERSE;
        public DcMotorSimple.Direction backRightDirection = DcMotorSimple.Direction.FORWARD;
    }

    public static class PinpointSettings {
        public String name = "pinpoint";
        public GoBildaPinpointDriver.GoBildaOdometryPods podType = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        public double xPodOffset = -6.5;
        public double yPodOffset = 1.25;
        public GoBildaPinpointDriver.EncoderDirection xPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public GoBildaPinpointDriver.EncoderDirection yPodDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
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
    public static PinpointSettings pinpoint = new PinpointSettings();
    public static ForesightSettings foresight = new ForesightSettings();

    public static MecanumConfig drivetrainConfig() {
        return new MecanumConfig(c -> {
            c.frontLeftName.set(mecanum.frontLeftName);
            c.frontRightName.set(mecanum.frontRightName);
            c.backLeftName.set(mecanum.backLeftName);
            c.backRightName.set(mecanum.backRightName);
            c.frontLeftDirection.set(mecanum.frontLeftDirection);
            c.frontRightDirection.set(mecanum.frontRightDirection);
            c.backLeftDirection.set(mecanum.backLeftDirection);
            c.backRightDirection.set(mecanum.backRightDirection);
        });
    }

    public static PinpointConfig localizerConfig() {
        return new PinpointConfig(c -> {
            c.name.set(pinpoint.name);
            c.podType.set(pinpoint.podType);
            c.xPodOffset.set(pinpoint.xPodOffset);
            c.yPodOffset.set(pinpoint.yPodOffset);
            c.xPodDirection.set(pinpoint.xPodDirection);
            c.yPodDirection.set(pinpoint.yPodDirection);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
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

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig()),
                new Mecanum(hardwareMap, drivetrainConfig()),
                new Foresight(foresightConfig()));
    }
}
