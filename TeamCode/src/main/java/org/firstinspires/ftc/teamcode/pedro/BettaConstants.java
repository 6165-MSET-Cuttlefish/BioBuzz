package org.firstinspires.ftc.teamcode.pedro;

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

public class BettaConstants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.frontRightName.set("fr");
        c.backLeftName.set("bl");
        c.backRightName.set("br");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-6.5);
        c.yPodOffset.set(1.25);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    /*public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.5104410832606671);
                Controller secondaryTranslationalForward = Controller.proportional(0.1885943410278743);
                Controller primaryTranslationalLateral = Controller.proportional(1.4702533330349719);
                Controller secondaryTranslationalLateral = Controller.proportional(0.5432193206638247);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.014785415249391384));
                c.brake.set(Controller.proportionalFeedforward(0.012567602961982676));

                c.headingFeedback.set(Controller.proportional(6.585528552262536));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06076848083880419, 0.006276081163690411));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10935402334154008, 0.09088664614818394));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001877290952712228, 0.001868833037798977));

                c.maxAchievableForwardVelocity.set(68.203704903592);
                c.maxAchievableStrafeVelocity.set(56.86699787940904);
                c.naturalForwardDeceleration.set(52.64447242735547);
                c.naturalStrafeDeceleration.set(82.55509035843839);
            }
    ); */
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.5101982626923285);
                Controller secondaryTranslationalForward = Controller.proportional(0.18850462531615828);
                Controller primaryTranslationalLateral = Controller.proportional(1.1272740084866855);
                Controller secondaryTranslationalLateral = Controller.proportional(0.4164976248195712);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013988930617921795));
                c.brake.set(Controller.proportionalFeedforward(0.011890591025233526));

                c.headingFeedback.set(Controller.proportional(5.618075516287591));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04586686604931309, 0.00885564987142792));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11619749704491415, 0.03999416772644396));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001387133564003021, 0.0027934483235117686));

                c.maxAchievableForwardVelocity.set(73.00734154443577);
                c.maxAchievableStrafeVelocity.set(60.048785872434564);
                c.naturalForwardDeceleration.set(61.973506354027336);
                c.naturalStrafeDeceleration.set(78.27862874148664);
            }
    );

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}
