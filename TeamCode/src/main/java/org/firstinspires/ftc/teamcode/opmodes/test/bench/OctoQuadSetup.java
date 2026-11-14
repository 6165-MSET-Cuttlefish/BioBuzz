package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.architecture.hardware.GuardedOctoQuadLocalizer;
import org.firstinspires.ftc.teamcode.opmodes.test.OctoQuadLocalizerTest;
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
        return localizerConfig(BenchIO.params());
    }

    static OctoQuadConfig localizerConfig(Map<String, Object> params) {
        int portX = (int) BenchIO.param(params, "portX", OctoQuadLocalizerTest.portX);
        int portY = (int) BenchIO.param(params, "portY", OctoQuadLocalizerTest.portY);
        return new OctoQuadConfig(c -> {
            c.name.set(OctoQuadLocalizerTest.name);
            c.xPodPort.set(portX);
            c.yPodPort.set(portY);
            c.xPodDirection.set(OctoQuadLocalizerTest.directionX);
            c.yPodDirection.set(OctoQuadLocalizerTest.directionY);
            c.encoderResolutionUnit.set(DistanceUnit.MM);
            c.ticksPerUnit.set(OctoQuadLocalizerTest.ticksPerMmX);
            c.offsetUnits.set(DistanceUnit.MM);
            c.xPodOffset.set(-OctoQuadLocalizerTest.tcpOffsetMmY);
            c.yPodOffset.set(-OctoQuadLocalizerTest.tcpOffsetMmX);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.headingScalar.set(OctoQuadLocalizerTest.imuHeadingScalar);
            c.localizerVelocityIntervalMS.set(OctoQuadLocalizerTest.velocityIntervalMs);
            c.i2cRecoveryMode.set(I2C_RECOVERY);
        });
    }

    static boolean guarded(Map<String, Object> params) {
        double guard = BenchIO.param(params, "guard", 1);
        if (guard != 0 && guard != 1) throw new IllegalArgumentException("bench param guard must be 0 or 1, was " + guard);
        return guard == 1;
    }

    static Follower create(HardwareMap hardwareMap) {
        Map<String, Object> params = BenchIO.params();
        OctoQuadConfig config = localizerConfig(params);
        Localizer localizer = guarded(params) ? new GuardedOctoQuadLocalizer(hardwareMap, config)
                : new OctoQuadLocalizer(hardwareMap, config);
        return new Follower(localizer,
                new Mecanum(hardwareMap, BettaConstants.drivetrainConfig()),
                new Foresight(BettaConstants.foresightConfig()));
    }

    static Map<String, Object> params() {
        Map<String, Object> runner = BenchIO.params();
        OctoQuadConfig c = localizerConfig(runner);
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("localizer", guarded(runner) ? "GuardedOctoQuadLocalizer" : "Pedro's OctoQuadLocalizer");
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

    static GuardedOctoQuadLocalizer guard(Follower follower) {
        if (!(follower.localizer instanceof GuardedOctoQuadLocalizer)) {
            throw new IllegalStateException("the follower's localizer is a " + follower.localizer.getClass().getName()
                    + ", not GuardedOctoQuadLocalizer");
        }
        return (GuardedOctoQuadLocalizer) follower.localizer;
    }

    // Pedro's OctoQuadLocalizer keeps the block it reads into private.
    static OctoQuad.LocalizerDataBlock lastRead(Follower follower) {
        if (follower.localizer instanceof GuardedOctoQuadLocalizer) return guard(follower).lastRead();
        if (!(follower.localizer instanceof OctoQuadLocalizer)) {
            throw new IllegalStateException("the follower's localizer is a " + follower.localizer.getClass().getName()
                    + ", not an OctoQuad localizer");
        }
        try {
            Field block = OctoQuadLocalizer.class.getDeclaredField("localizer");
            block.setAccessible(true);
            return (OctoQuad.LocalizerDataBlock) block.get(follower.localizer);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new IllegalStateException("Pedro's OctoQuadLocalizer no longer keeps its LocalizerDataBlock in a field "
                    + "named localizer", e);
        }
    }
}
