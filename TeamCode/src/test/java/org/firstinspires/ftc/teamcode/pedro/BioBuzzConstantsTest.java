package org.firstinspires.ftc.teamcode.pedro;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.architecture.hardware.GuardedOctoQuadLocalizer;
import org.firstinspires.ftc.teamcode.opmodes.tele.BioBuzzTele;
import org.firstinspires.ftc.teamcode.opmodes.test.BioBuzzShooterTest;
import org.firstinspires.ftc.teamcode.opmodes.test.CameraModuleTest;
import org.firstinspires.ftc.teamcode.opmodes.test.CellTipTest;
import org.firstinspires.ftc.teamcode.opmodes.test.MockAuto;
import org.firstinspires.ftc.teamcode.opmodes.test.SotmTest;
import org.firstinspires.ftc.teamcode.opmodes.test.TurretAimTest;
import org.firstinspires.ftc.teamcode.opmodes.test.auto.BallCollectionAuto;
import org.firstinspires.ftc.teamcode.opmodes.test.auto.BallCollectionTest;
import org.firstinspires.ftc.teamcode.opmodes.test.auto.CloseFlowerAuto;
import org.firstinspires.ftc.teamcode.opmodes.test.auto.ForwardBackTest;
import org.firstinspires.ftc.teamcode.opmodes.test.auto.VisionBallCollectionTest;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

public class BioBuzzConstantsTest {
    private static final double EPS = 1e-9;
    private static final float F_EPS = 1e-4f;

    private boolean savedMeasured;
    private BioBuzzConstants.OctoQuadSettings savedOctoQuad;

    @Before
    public void saveStatics() {
        savedMeasured = BioBuzzConstants.measured;
        savedOctoQuad = BioBuzzConstants.octoQuad;
        BioBuzzConstants.octoQuad = new BioBuzzConstants.OctoQuadSettings();
    }

    @After
    public void restoreStatics() {
        BioBuzzConstants.measured = savedMeasured;
        BioBuzzConstants.octoQuad = savedOctoQuad;
    }

    @Test
    public void theSettingsStartUnmeasured() {
        assertFalse(BioBuzzConstants.measured);
    }

    @Test
    public void theGateIsNotAPublicFieldSoTheDashboardCannotListIt() throws Exception {
        for (Field f : BioBuzzConstants.class.getFields()) assertFalse(f.getName(), f.getName().equals("measured"));
        assertFalse(Modifier.isPublic(BioBuzzConstants.class.getDeclaredField("measured").getModifiers()));
    }

    @Test
    public void everyOctoQuadSettingReachesPedrosConfig() {
        BioBuzzConstants.OctoQuadSettings s = BioBuzzConstants.octoQuad;
        s.name = "odo";
        s.forwardPodChannel = 1;
        s.strafePodChannel = 0;
        s.forwardPodDirection = OctoQuad.EncoderDirection.FORWARD;
        s.strafePodDirection = OctoQuad.EncoderDirection.REVERSE;
        s.countsPerMm = 13.26291192;
        s.forwardPodLeftOfCentreIn = 2.5;
        s.strafePodForwardOfCentreIn = -3.25;
        s.headingScalar = 0.998;
        s.velocityIntervalMs = 40;

        OctoQuadConfig c = BioBuzzConstants.localizerConfig();

        assertEquals("odo", c.name.get());
        assertEquals(1, (int) c.xPodPort.get());
        assertEquals(0, (int) c.yPodPort.get());
        assertEquals(OctoQuad.EncoderDirection.FORWARD, c.xPodDirection.get());
        assertEquals(OctoQuad.EncoderDirection.REVERSE, c.yPodDirection.get());
        assertEquals(DistanceUnit.MM, c.encoderResolutionUnit.get());
        assertEquals(13.26291192, c.ticksPerUnit.get(), EPS);
        assertEquals(DistanceUnit.INCH, c.offsetUnits.get());
        assertEquals(2.5, c.xPodOffset.get(), EPS);
        assertEquals(-3.25, c.yPodOffset.get(), EPS);
        assertEquals(DistanceUnit.INCH, c.globalDistanceUnit.get());
        assertEquals(0.998, c.headingScalar.get(), EPS);
        assertEquals(40, (int) c.localizerVelocityIntervalMS.get());
        assertEquals(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR, c.i2cRecoveryMode.get());
    }

    @Test
    public void theStartingValuesWriteTheBettaBotsOctoQuadNumbersOnChannelsZeroAndOne() {
        FakeOctoQuad board = new FakeOctoQuad();
        BioBuzzConstants.localizer(new FakeHardwareMap().with("octoquad", board.device));

        assertEquals(Arrays.asList("0 REVERSE", "1 REVERSE"), board.directions);
        assertEquals(0, board.params[0]);
        assertEquals(1, board.params[1]);
        assertEquals(19.89436789f, (float) board.params[2], F_EPS);
        assertEquals(19.89436789f, (float) board.params[3], F_EPS);
        assertEquals(-34.13125f, (float) board.params[4], F_EPS);
        assertEquals(157.1625f, (float) board.params[5], F_EPS);
        assertEquals(1.0085f, (float) board.params[6], F_EPS);
        assertEquals(25, board.params[7]);
        assertEquals(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR, board.recovery);
        assertEquals(1, board.resets);
    }

    @Test
    public void podOffsetsBecomeTheBoardsOffsetFromThePodsCrossingToTheCentre() {
        BioBuzzConstants.octoQuad.forwardPodLeftOfCentreIn = 4;
        BioBuzzConstants.octoQuad.strafePodForwardOfCentreIn = -2;
        FakeOctoQuad board = new FakeOctoQuad();
        BioBuzzConstants.localizer(new FakeHardwareMap().with("octoquad", board.device));

        assertEquals(2 * 25.4f, (float) board.params[4], F_EPS);
        assertEquals(-4 * 25.4f, (float) board.params[5], F_EPS);
    }

    @Test
    public void poseIsInInchesWithHeadingNormalisedLikePedros() {
        FakeOctoQuad board = new FakeOctoQuad();
        GuardedOctoQuadLocalizer localizer = BioBuzzConstants.localizer(new FakeHardwareMap().with("octoquad", board.device));
        board.running(254, -508, (float) (-Math.PI / 2), 127, 0, 0.5f);

        localizer.update();

        Pose pose = localizer.state().pose();
        assertEquals(10, pose.x(), 1e-6);
        assertEquals(-20, pose.y(), 1e-6);
        assertEquals(3 * Math.PI / 2, pose.heading(), 1e-6);
        Velocity v = localizer.state().velocity();
        assertEquals(5, v.vx, 1e-6);
        assertEquals(0, v.vy, 1e-6);
        assertEquals(0.5, v.omega, 1e-6);
    }

    @Test
    public void aStartPoseSetBeforeTheImuIsReadyIsWrittenOnceItIs() {
        FakeOctoQuad board = new FakeOctoQuad();
        GuardedOctoQuadLocalizer localizer = BioBuzzConstants.localizer(new FakeHardwareMap().with("octoquad", board.device));
        board.status = OctoQuad.LocalizerStatus.CALIBRATING_IMU;

        localizer.setPose(new Pose(56.5205, 19.2527, Math.PI / 2));
        localizer.update();
        assertTrue(board.poseWrites.isEmpty());
        assertEquals(56.5205, localizer.state().pose().x(), EPS);

        board.running(0, 0, 0, 0, 0, 0);
        localizer.update();

        assertEquals(Arrays.asList("1435 489 " + (float) (Math.PI / 2)), board.poseWrites);
        assertEquals(56.5205, localizer.state().pose().x(), EPS);
        assertEquals(19.2527, localizer.state().pose().y(), EPS);
        assertEquals(Math.PI / 2, localizer.state().pose().heading(), EPS);
    }

    @Test
    public void aPoseSetWhileRunningGoesStraightToTheBoardInWholeMillimetres() {
        FakeOctoQuad board = new FakeOctoQuad();
        GuardedOctoQuadLocalizer localizer = BioBuzzConstants.localizer(new FakeHardwareMap().with("octoquad", board.device));
        board.running(0, 0, 0, 0, 0, 0);
        localizer.update();

        localizer.setPose(new Pose(72, 134, Math.PI));

        assertEquals(Arrays.asList("1828 3403 " + (float) Math.PI), board.poseWrites);
    }

    @Test
    public void stopEndsTheWaitForRunningWithoutAnError() {
        FakeOctoQuad board = new FakeOctoQuad();
        GuardedOctoQuadLocalizer localizer = BioBuzzConstants.localizer(new FakeHardwareMap().with("octoquad", board.device));
        int[] polls = {0};

        long start = System.nanoTime();
        localizer.awaitRunning(() -> ++polls[0] > 3);

        assertTrue((System.nanoTime() - start) / 1e6 + " ms", System.nanoTime() - start < 1_000_000_000L);
        assertEquals(4, polls[0]);
        assertEquals("CALIBRATING_IMU", localizer.status());
    }

    @Test
    public void untilMeasuredAPathOpModeStopsWithASentenceSayingWhatToMeasure() {
        BioBuzzConstants.measured = false;
        try {
            BioBuzzConstants.requireMeasured(BioBuzzConstants.displayName(ForwardBackTest.class));
            fail("expected the gate to stop Forward Back Test");
        } catch (IllegalStateException e) {
            String m = e.getMessage();
            assertTrue(m, m.startsWith("Forward Back Test lets Pedro drive the robot"));
            assertTrue(m, m.contains("drive directions, odometry pods, IMU heading scale and Foresight gains"));
            assertTrue(m, m.contains("docs/odometry-tuning.md"));
            assertTrue(m, m.contains("BioBuzzConstants.measured"));
            assertEquals(1, m.split("[.:] [A-Z]").length);
        }
    }

    @Test
    public void createStopsAPathOpModeBeforeTouchingAnyHardware() {
        BioBuzzConstants.measured = false;
        FakeOctoQuad board = new FakeOctoQuad();
        ForwardBackTest opMode = new ForwardBackTest();
        opMode.hardwareMap = robotMap(board);
        try {
            BioBuzzConstants.create(opMode);
            fail("expected the gate to stop Forward Back Test");
        } catch (IllegalStateException e) {
            assertTrue(e.getMessage(), e.getMessage().startsWith("Forward Back Test lets Pedro drive the robot"));
        }
        assertEquals(0, board.resets);
    }

    @Test
    public void createBuildsTheOctoQuadFollowerForAHandDrivenOpModeBeforeMeasuring() {
        BioBuzzConstants.measured = false;
        FakeOctoQuad board = new FakeOctoQuad();
        BioBuzzTele opMode = new BioBuzzTele();
        opMode.hardwareMap = robotMap(board);

        Follower follower = BioBuzzConstants.create(opMode);

        assertTrue(follower.localizer instanceof GuardedOctoQuadLocalizer);
        assertEquals(1, board.resets);
    }

    private static HardwareMap robotMap(FakeOctoQuad board) {
        return robotMap(board, new ArrayList<>());
    }

    private static HardwareMap robotMap(FakeOctoQuad board, List<String> motorCalls) {
        FakeHardwareMap map = new FakeHardwareMap().with("octoquad", board.device);
        for (String motor : new String[]{"fl", "fr", "bl", "br"}) {
            map.with(motor, (DcMotorEx) Proxy.newProxyInstance(BioBuzzConstantsTest.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
                        if (!method.getDeclaringClass().equals(Object.class)) motorCalls.add(motor + "." + method.getName());
                        Class<?> r = method.getReturnType();
                        if (r == boolean.class) return false;
                        if (r == int.class) return 0;
                        if (r == double.class) return 0.0;
                        if (method.getName().equals("hashCode")) return System.identityHashCode(proxy);
                        if (method.getName().equals("equals")) return proxy == args[0];
                        return null;
                    }));
        }
        return map;
    }

    @Test
    public void onceMeasuredTheGateIsOpen() {
        BioBuzzConstants.measured = true;
        BioBuzzConstants.requireMeasured("Close Flower Auto");
    }

    @Test
    public void everyOpModeWherePedroDrivesIsGatedAndOnlyHandDrivenOnesAreNot() {
        List<Class<?>> pedroDrives = Arrays.asList(ForwardBackTest.class, CloseFlowerAuto.class, BallCollectionAuto.class,
                MockAuto.class, BallCollectionTest.class, VisionBallCollectionTest.class);
        List<Class<?>> handDriven = Arrays.asList(BioBuzzTele.class, SotmTest.class, TurretAimTest.class, CellTipTest.class,
                BioBuzzShooterTest.class, CameraModuleTest.class);
        List<String> wrong = new ArrayList<>();
        for (Class<?> c : pedroDrives) if (c.isAnnotationPresent(PedroNeverDrives.class)) wrong.add(c.getSimpleName());
        for (Class<?> c : handDriven) if (!c.isAnnotationPresent(PedroNeverDrives.class)) wrong.add(c.getSimpleName());
        assertTrue(wrong.toString(), wrong.isEmpty());
        for (Class<?> c : pedroDrives) {
            assertTrue(c.getSimpleName(), c.isAnnotationPresent(Autonomous.class) || c.isAnnotationPresent(TeleOp.class));
        }
    }

    @PedroNeverDrives
    static class HandDriven {
    }

    static class SubclassOfHandDriven extends HandDriven {
    }

    @Test
    public void theMarkIsNotInheritedSoASubclassIsGatedUntilMarkedItself() {
        assertTrue(HandDriven.class.isAnnotationPresent(PedroNeverDrives.class));
        assertFalse(SubclassOfHandDriven.class.isAnnotationPresent(PedroNeverDrives.class));
    }

    @Test
    public void theGateNamesAnOpModeByItsDriverStationName() {
        assertEquals("Forward Back Test", BioBuzzConstants.displayName(ForwardBackTest.class));
        assertEquals("Ball Collection", BioBuzzConstants.displayName(BallCollectionTest.class));
        assertEquals("HandDriven", BioBuzzConstants.displayName(HandDriven.class));
    }

    @SuppressWarnings("unchecked")
    private static <T> T testsSupplier(String field) throws Exception {
        Object tests = Tuning.tests();
        Field f = tests.getClass().getDeclaredField(field);
        f.setAccessible(true);
        return (T) f.get(tests);
    }

    private static void assertAutoTuneRefusal(Throwable e) {
        assertTrue(String.valueOf(e), e instanceof IllegalStateException);
        assertTrue(e.getMessage(), e.getMessage().startsWith("AutoTune's Tests lets Pedro drive the robot"));
    }

    @Test
    public void autoTunesTestsWontBuildTheDrivetrainUntilMeasured() throws Exception {
        Function<HardwareMap, Drivetrain> drivetrain = testsSupplier("drivetrainFunction");
        List<String> motorCalls = new ArrayList<>();
        HardwareMap map = robotMap(new FakeOctoQuad(), motorCalls);

        BioBuzzConstants.measured = false;
        try {
            drivetrain.apply(map);
            fail("expected AutoTune's Tests to refuse the drivetrain");
        } catch (IllegalStateException e) {
            assertAutoTuneRefusal(e);
        }
        assertTrue(motorCalls.toString(), motorCalls.isEmpty());

        BioBuzzConstants.measured = true;
        assertTrue(drivetrain.apply(map) instanceof Mecanum);
    }

    @Test
    public void autoTunesTestsWontBuildTheFollowersAlgorithmUntilMeasured() throws Exception {
        Supplier<Algorithm> supplier = testsSupplier("algorithmSupplier");

        BioBuzzConstants.measured = false;
        try {
            supplier.get();
            fail("expected AutoTune's Tests to refuse the algorithm");
        } catch (IllegalStateException e) {
            assertAutoTuneRefusal(e);
        }
        BioBuzzConstants.measured = true;
        assertTrue(supplier.get() != null);
    }

    @Test(timeout = 10_000)
    public void autoTunesOdometryTestStopsBeforeAnyDriveMotorUntilMeasured() throws Exception {
        FakeOctoQuad board = new FakeOctoQuad();
        board.running(0, 0, 0, 0, 0, 0);
        List<String> motorCalls = new ArrayList<>();
        Class<?> odometry = Class.forName("org.firstinspires.ftc.teamcode.pedro.procedures.TestsOdometry");
        Constructor<?> constructor = odometry.getDeclaredConstructor(Function.class, Function.class);
        constructor.setAccessible(true);
        LinearOpMode test = (LinearOpMode) constructor.newInstance(testsSupplier("drivetrainFunction"),
                testsSupplier("localizerFunction"));
        test.hardwareMap = robotMap(board, motorCalls);
        Method run = odometry.getDeclaredMethod("runTuningOpMode");
        run.setAccessible(true);

        BioBuzzConstants.measured = false;
        try {
            run.invoke(test);
            fail("expected AutoTune's Odometry Test to stop before it drives");
        } catch (InvocationTargetException e) {
            assertAutoTuneRefusal(e.getCause());
        }
        assertTrue(motorCalls.toString(), motorCalls.isEmpty());
    }

    @Test
    public void autoTunesPoseTestStillReadsTheOctoQuadBeforeMeasuring() throws Exception {
        Function<HardwareMap, Localizer> localizer = testsSupplier("localizerFunction");
        FakeOctoQuad board = new FakeOctoQuad();
        board.running(254, 0, 0, 0, 0, 0);

        BioBuzzConstants.measured = false;
        Localizer built = localizer.apply(robotMap(board, new ArrayList<>()));

        assertTrue(built instanceof GuardedOctoQuadLocalizer);
        built.update();
        assertEquals(10, built.pose().x(), 1e-6);
    }

    @Test
    public void autoTuneDoesNotListTheVendoredForesightTuner() {
        List<String> listed = new ArrayList<>();
        for (Method m : Tuning.class.getDeclaredMethods()) {
            Tuner tuner = m.getAnnotation(Tuner.class);
            if (tuner != null) listed.add(tuner.name());
        }
        assertTrue(listed.toString(), listed.contains("Tests"));
        assertFalse(listed.toString(), listed.contains("Foresight Tuner"));
    }

    private static final class FakeHardwareMap extends HardwareMap {
        private final java.util.Map<String, HardwareDevice> devices = new java.util.HashMap<>();

        FakeHardwareMap() {
            super(null, null);
        }

        FakeHardwareMap with(String name, HardwareDevice device) {
            devices.put(name, device);
            return this;
        }

        @Override
        public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
            HardwareDevice device = devices.get(deviceName);
            if (!classOrInterface.isInstance(device)) {
                throw new IllegalArgumentException("no " + classOrInterface.getSimpleName() + " named " + deviceName);
            }
            return classOrInterface.cast(device);
        }
    }

    private static final class FakeOctoQuad {
        OctoQuad.LocalizerStatus status = OctoQuad.LocalizerStatus.CALIBRATING_IMU;
        short x, y, vx, vy;
        float heading, omega;
        final List<String> directions = new ArrayList<>();
        final List<String> poseWrites = new ArrayList<>();
        Object[] params;
        OctoQuad.I2cRecoveryMode recovery;
        int resets;

        final OctoQuad device = (OctoQuad) Proxy.newProxyInstance(FakeOctoQuad.class.getClassLoader(),
                new Class<?>[]{OctoQuad.class}, (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "getChipId": return OctoQuad.OCTOQUAD_CHIP_ID;
                        case "setSingleEncoderDirection": directions.add(args[0] + " " + args[1]); return null;
                        case "setAllLocalizerParameters": params = args; return null;
                        case "setI2cRecoveryMode": recovery = (OctoQuad.I2cRecoveryMode) args[0]; return null;
                        case "resetLocalizerAndCalibrateIMU": resets++; return null;
                        case "getLocalizerStatus": return status;
                        case "readLocalizerData": fill((OctoQuad.LocalizerDataBlock) args[0]); return null;
                        case "setLocalizerPose": poseWrites.add(args[0] + " " + args[1] + " " + args[2]); return null;
                        case "getConnectionInfo": return "I2C bus 2";
                        case "hashCode": return System.identityHashCode(proxy);
                        case "equals": return proxy == args[0];
                        case "toString": return "FakeOctoQuad";
                        default: throw new UnsupportedOperationException(method.getName());
                    }
                });

        void running(int xMm, int yMm, float headingRad, int vxMmS, int vyMmS, float omegaRadS) {
            status = OctoQuad.LocalizerStatus.RUNNING;
            x = (short) xMm;
            y = (short) yMm;
            heading = headingRad;
            vx = (short) vxMmS;
            vy = (short) vyMmS;
            omega = omegaRadS;
        }

        private void fill(OctoQuad.LocalizerDataBlock data) {
            data.crcOk = true;
            data.localizerStatus = status;
            data.posX_mm = x;
            data.posY_mm = y;
            data.heading_rad = heading;
            data.velX_mmS = vx;
            data.velY_mmS = vy;
            data.velHeading_radS = omega;
        }
    }
}
