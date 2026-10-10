package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.octoquad.OctoQuadLocalizerTest;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

import java.lang.reflect.Field;
import java.util.LinkedHashMap;
import java.util.Map;

final class OctoQuadSetup {
    static final OctoQuad.I2cRecoveryMode I2C_RECOVERY = OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR;

    private OctoQuadSetup() {}

    static final class OctoQuadRobot extends Robot {
        OctoQuadRobot(EnhancedOpMode opMode) {
            super(opMode);
        }

        @Override
        protected Follower createFollower(HardwareMap hardwareMap) {
            return create(hardwareMap);
        }

        @Override
        protected void initializeGameModules() {}
    }

    static OctoQuadConfig localizerConfig() {
        return new OctoQuadConfig(c -> {
            c.name.set(OctoQuadLocalizerTest.name);
            c.xPodPort.set(OctoQuadLocalizerTest.portX);
            c.yPodPort.set(OctoQuadLocalizerTest.portY);
            c.xPodDirection.set(OctoQuadLocalizerTest.directionX);
            c.yPodDirection.set(OctoQuadLocalizerTest.directionY);
            c.encoderResolutionUnit.set(DistanceUnit.MM);
            c.ticksPerUnit.set(OctoQuadLocalizerTest.ticksPerMmX);
            c.offsetUnits.set(DistanceUnit.MM);
            c.xPodOffset.set(-OctoQuadLocalizerTest.tcpOffsetMmX);
            c.yPodOffset.set(-OctoQuadLocalizerTest.tcpOffsetMmY);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.headingScalar.set(OctoQuadLocalizerTest.imuHeadingScalar);
            c.localizerVelocityIntervalMS.set(OctoQuadLocalizerTest.velocityIntervalMs);
            c.i2cRecoveryMode.set(I2C_RECOVERY);
        });
    }

    static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new OctoQuadLocalizer(hardwareMap, localizerConfig()),
                new Mecanum(hardwareMap, BettaConstants.drivetrainConfig()),
                new Foresight(BettaConstants.foresightConfig()));
    }

    static Map<String, Object> params() {
        OctoQuadConfig c = localizerConfig();
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("portX", c.xPodPort.get());
        m.put("portY", c.yPodPort.get());
        m.put("directionX", c.xPodDirection.get().name());
        m.put("directionY", c.yPodDirection.get().name());
        m.put("ticksPerMm", c.ticksPerUnit.get());
        if (OctoQuadLocalizerTest.ticksPerMmY != OctoQuadLocalizerTest.ticksPerMmX) {
            m.put("ticksPerMmYIgnored", OctoQuadLocalizerTest.ticksPerMmY);
        }
        m.put("xPodOffsetMm", c.xPodOffset.get());
        m.put("yPodOffsetMm", c.yPodOffset.get());
        m.put("headingScalar", c.headingScalar.get());
        m.put("velocityIntervalMs", c.localizerVelocityIntervalMS.get());
        m.put("i2cRecoveryMode", c.i2cRecoveryMode.get().name());
        return m;
    }

    static OctoQuad.LocalizerDataBlock localizerData(Follower follower) {
        if (!(follower.localizer instanceof OctoQuadLocalizer)) {
            throw new IllegalStateException("the follower's localizer is a " + follower.localizer.getClass().getName()
                    + ", not Pedro's OctoQuadLocalizer");
        }
        try {
            Field f = OctoQuadLocalizer.class.getDeclaredField("localizer");
            f.setAccessible(true);
            return (OctoQuad.LocalizerDataBlock) f.get(follower.localizer);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new IllegalStateException("Pedro's OctoQuadLocalizer has no readable localizer field; the OctoQuad benches need updating", e);
        }
    }
}
