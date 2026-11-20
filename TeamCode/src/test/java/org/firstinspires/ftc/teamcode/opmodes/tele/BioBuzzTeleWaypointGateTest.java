package org.firstinspires.ftc.teamcode.opmodes.tele;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzOpMode;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzRobot;
import org.firstinspires.ftc.teamcode.biobuzz.RobotActions;
import org.firstinspires.ftc.teamcode.biobuzz.modules.Camera;
import org.firstinspires.ftc.teamcode.biobuzz.modules.CellTipCamera;
import org.firstinspires.ftc.teamcode.biobuzz.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.pedro.BioBuzzConstants;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class BioBuzzTeleWaypointGateTest {
    private static final double EPS = 1e-9;

    private boolean savedMeasured;
    private AllianceColor savedAlliance;
    private final Map<String, Double> wheelPowers = new HashMap<>();
    private final List<DrivePowers> pedroDrives = new ArrayList<>();
    private final Map<String, String> shown = new HashMap<>();
    private final List<Integer> blips = new ArrayList<>();
    private Pose pose = new Pose(0, 0, 0);
    private Drivetrain drivetrain;
    private BioBuzzTele tele;

    private static final class VisionStates extends Module {
        VisionStates() {
            setStates(Camera.VisionState.ENABLED, CellTipCamera.VisionState.ENABLED);
        }

        @Override protected void initStates() { }
        @Override protected void read() { }
        @Override protected void write() { }
        @Override public void stop() { }
    }

    @Before
    public void build() throws Exception {
        savedMeasured = measuredField().getBoolean(null);
        savedAlliance = Context.allianceColor;
        Context.allianceColor = AllianceColor.RED;
        FieldConfig.setSymmetry(BioBuzzField.SYMMETRY);
        State.clearModuleBindings();
        Scheduler.reset();

        Map<String, HardwareDevice> devices = new HashMap<>();
        for (String motor : new String[]{Drivetrain.FRONT_LEFT, Drivetrain.BACK_LEFT, Drivetrain.FRONT_RIGHT, Drivetrain.BACK_RIGHT}) {
            devices.put(motor, (DcMotorEx) Proxy.newProxyInstance(getClass().getClassLoader(), new Class<?>[]{DcMotorEx.class},
                    (proxy, method, args) -> {
                        if (method.getName().equals("setPower")) wheelPowers.put(motor, (Double) args[0]);
                        Class<?> r = method.getReturnType();
                        if (r == boolean.class) return false;
                        if (r == int.class) return 0;
                        if (r == double.class) return 0.0;
                        if (method.getName().equals("hashCode")) return System.identityHashCode(proxy);
                        if (method.getName().equals("equals")) return proxy == args[0];
                        return null;
                    }));
        }
        devices.put("floodgate", new AnalogInput(null, 0));
        HardwareMap map = new HardwareMap(null, null) {
            @Override
            public <T> T get(Class<? extends T> type, String name) {
                return type.cast(devices.get(name));
            }
        };
        Follower follower = new Follower(new Localizer() {
            @Override public void setPose(Pose p) { pose = p; }
            @Override public Pose pose() { return pose; }
            @Override public MotionState state() { return MotionState.zero().withPose(pose); }
            @Override public void update() { }
            @Override public void reset() { }
        }, new com.pedropathing.drivetrain.Drivetrain() {
            @Override public void drive(DrivePowers powers, boolean manual) { pedroDrives.add(powers); }
            @Override public double maxScaling(DrivePowers a, DrivePowers b) { return 1; }
            @Override public void stop() { }
            @Override public void stop(boolean brake) { }
            @Override public Map<String, Object> debug() { return new HashMap<>(); }
            @Override public double interpolateVelocity(double a, double b, double c) { return 0; }
        }, null);
        drivetrain = new Drivetrain(map, follower);

        BioBuzzRobot robot = unconstructed(BioBuzzRobot.class);
        Field followerField = Robot.class.getDeclaredField("follower");
        followerField.setAccessible(true);
        followerField.set(robot, follower);
        robot.drivetrain = drivetrain;
        robot.actions = new RobotActions(robot);

        tele = new BioBuzzTele();
        Field robotField = BioBuzzOpMode.class.getDeclaredField("robot");
        robotField.setAccessible(true);
        robotField.set(tele, robot);
        tele.gamepad1 = new Gamepad() {
            @Override public void rumbleBlips(int count) { blips.add(count); }
            @Override public void rumble(int durationMs) { }
        };
        tele.gamepad2 = new Gamepad();
        tele.telemetry = (Telemetry) Proxy.newProxyInstance(getClass().getClassLoader(), new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> {
                    if (method.getName().equals("addData") && args.length == 2) {
                        shown.put((String) args[0], String.valueOf(args[1]));
                    } else if (method.getName().equals("addData") && args.length == 3) {
                        shown.put((String) args[0], String.format((String) args[1], (Object[]) args[2]));
                    }
                    if (method.getName().equals("hashCode")) return System.identityHashCode(proxy);
                    if (method.getName().equals("equals")) return proxy == args[0];
                    return null;
                });

        new VisionStates();
        tele.initialize();
        tele.onStart();
    }

    @After
    public void restore() throws Exception {
        measuredField().setBoolean(null, savedMeasured);
        Context.allianceColor = savedAlliance;
        FieldConfig.clear();
        State.clearModuleBindings();
        Scheduler.reset();
    }

    private static Field measuredField() throws Exception {
        Field f = BioBuzzConstants.class.getDeclaredField("measured");
        f.setAccessible(true);
        return f;
    }

    private static <T> T unconstructed(Class<T> type) throws Exception {
        Class<?> unsafeClass = Class.forName("sun.misc.Unsafe");
        Field theUnsafe = unsafeClass.getDeclaredField("theUnsafe");
        theUnsafe.setAccessible(true);
        return type.cast(unsafeClass.getMethod("allocateInstance", Class.class).invoke(theUnsafe.get(null), type));
    }

    private void loopWith(boolean a, boolean lt) {
        Gamepad state = new Gamepad();
        state.a = a;
        state.left_trigger = lt ? 1 : 0;
        tele.gamepad1.copy(state);
        tele.gameLoop();
    }

    private void pressRb() {
        Gamepad state = new Gamepad();
        state.right_bumper = true;
        tele.gamepad1.copy(state);
        tele.gameLoop();
        tele.telemetry();
    }

    private void saveTheWaypointThenDriveAwayAndPressLt() {
        loopWith(true, false);
        loopWith(false, false);
        tele.telemetry();
        assertEquals("saved", shown.get("Waypoint Drive"));
        pose = new Pose(20, 20, 0);
        loopWith(false, true);
        tele.telemetry();
    }

    @Test
    public void untilMeasuredLtStartsNoDriveAndRumblesThreeBlips() throws Exception {
        measuredField().setBoolean(null, false);

        saveTheWaypointThenDriveAwayAndPressLt();
        for (int i = 0; i < 5; i++) {
            Scheduler.execute();
            loopWith(false, false);
        }
        tele.telemetry();

        assertEquals("TELE", shown.get("D1 Layer"));
        assertTrue(shown.get("Waypoint Drive"), shown.get("Waypoint Drive").contains("odometry to be measured"));
        assertEquals(1, blips.size());
        assertEquals(3, (int) blips.get(0));
        assertTrue("Pedro never got the drivetrain", drivetrain.isWriteEnabled());
        assertTrue(pedroDrives.toString(), pedroDrives.isEmpty());
        for (double power : wheelPowers.values()) assertEquals(0, power, EPS);
        for (double power : drivetrain.getMotorPowers()) assertEquals(0, power, EPS);
    }

    @Test
    public void onceMeasuredLtStartsTheDrive() throws Exception {
        measuredField().setBoolean(null, true);

        saveTheWaypointThenDriveAwayAndPressLt();

        assertEquals("WAYPOINT_DRIVE", shown.get("D1 Layer"));
        assertTrue(shown.get("Waypoint Drive"), shown.get("Waypoint Drive").startsWith("PLANNING"));
        assertFalse("the drive took the drivetrain for Pedro", drivetrain.isWriteEnabled());
        assertTrue(blips.toString(), blips.isEmpty());
    }

    @Test
    public void untilMeasuredRbLocksNothingAndRumblesThreeBlips() throws Exception {
        measuredField().setBoolean(null, false);

        pressRb();
        for (int i = 0; i < 5; i++) {
            Scheduler.execute();
            loopWith(false, false);
        }
        tele.telemetry();

        assertEquals("TELE", shown.get("D1 Layer"));
        assertTrue(shown.get("Position Lock"), shown.get("Position Lock").contains("odometry to be measured"));
        assertEquals(1, blips.size());
        assertEquals(3, (int) blips.get(0));
        assertTrue("Pedro never got the drivetrain", drivetrain.isWriteEnabled());
        assertTrue(pedroDrives.toString(), pedroDrives.isEmpty());
        for (double power : wheelPowers.values()) assertEquals(0, power, EPS);
        for (double power : drivetrain.getMotorPowers()) assertEquals(0, power, EPS);
    }

    @Test
    public void onceMeasuredRbLocksThePosition() throws Exception {
        measuredField().setBoolean(null, true);

        pressRb();

        assertEquals("POSITION_LOCK", shown.get("D1 Layer"));
        assertTrue(shown.get("Position Lock"), shown.get("Position Lock").startsWith("holding"));
        assertFalse("the lock took the drivetrain for Pedro", drivetrain.isWriteEnabled());
        assertTrue(blips.toString(), blips.isEmpty());
    }
}
