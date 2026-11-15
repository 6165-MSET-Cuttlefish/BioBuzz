package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public final class PinpointSetup {
    private PinpointSetup() {}

    public static class PinpointSettings {
        public String name = "pinpoint";
        public GoBildaPinpointDriver.GoBildaOdometryPods podType = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        // OctoQuadSetup's TCP offset points from the pods' TCP to the centre, so these are (-tcpOffsetMmY, -tcpOffsetMmX) in inches.
        public double xPodOffset = -6.1875;
        public double yPodOffset = 1.34375;
        public GoBildaPinpointDriver.EncoderDirection xPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public GoBildaPinpointDriver.EncoderDirection yPodDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    }

    public static PinpointSettings pinpoint = new PinpointSettings();

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

    public static PinpointLocalizer localizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig());
    }

    public static Follower create(HardwareMap hardwareMap) {
        return BenchRobot.benchFollower(hardwareMap, PinpointSetup::localizer);
    }
}
