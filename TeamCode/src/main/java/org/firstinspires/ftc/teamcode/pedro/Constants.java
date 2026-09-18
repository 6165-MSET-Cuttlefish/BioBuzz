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
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("br");
        c.frontRightName.set("bl");
        c.backLeftName.set("fr");
        c.backRightName.set("fl");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(5.827277476393332);
        c.yPodOffset.set(-0.7426906946137196);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    // UNTUNED placeholders, deliberately gentle — replace this whole block with the ForesightTuner output before trusting any path.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.4719416329804985);
                Controller secondaryTranslationalForward = Controller.proportional(0.17436982287360983);
                Controller primaryTranslationalLateral = Controller.proportional(0.9971716061004495);
                Controller secondaryTranslationalLateral = Controller.proportional(0.3684282635380746);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013866972213354378));
                c.brake.set(Controller.proportionalFeedforward(0.011786926381351221));

                c.headingFeedback.set(Controller.proportional(9.112070085860307));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.070083347404981, 0.007248182030146053));

                c.linearBrakeCoefficients.set(Matrix.diag(0.13963824883716847, 0.08368963017112342));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001174212492667325, 0.0015568967035917184));

                c.maxAchievableForwardVelocity.set(70.9821485813868);
                c.maxAchievableStrafeVelocity.set(50.45422508725285);
                c.naturalForwardDeceleration.set(29.42678794810608);
                c.naturalStrafeDeceleration.set(82.65319300265601);
            }
    );

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}
