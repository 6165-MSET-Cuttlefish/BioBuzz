package org.firstinspires.ftc.teamcode.biobuzz.modules;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.BioBuzzConstants;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.Map;

public class HeadingLockGateTest {
    private static final double EPS = 1e-9;

    private boolean savedMeasured;
    private final Map<String, Double> wheelPowers = new HashMap<>();
    private Pose pose = new Pose(0, 0, 0);
    private Drivetrain drivetrain;

    @Before
    public void build() throws Exception {
        savedMeasured = measuredField().getBoolean(null);
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
        drivetrain = new Drivetrain(map, new Follower(new Localizer() {
            @Override public void setPose(Pose p) { pose = p; }
            @Override public Pose pose() { return pose; }
            @Override public MotionState state() { return MotionState.zero().withPose(pose); }
            @Override public void update() { }
            @Override public void reset() { }
        }, new com.pedropathing.drivetrain.Drivetrain() {
            @Override public void drive(DrivePowers powers, boolean manual) { }
            @Override public double maxScaling(DrivePowers a, DrivePowers b) { return 1; }
            @Override public void stop() { }
            @Override public void stop(boolean brake) { }
            @Override public Map<String, Object> debug() { return new HashMap<>(); }
            @Override public double interpolateVelocity(double a, double b, double c) { return 0; }
        }, null));
    }

    @After
    public void restore() throws Exception {
        measuredField().setBoolean(null, savedMeasured);
    }

    private static Field measuredField() throws Exception {
        Field f = BioBuzzConstants.class.getDeclaredField("measured");
        f.setAccessible(true);
        return f;
    }

    private void lockThenTurnOffTheLockedHeadingWithNoStick() {
        pose = new Pose(0, 0, 0);
        drivetrain.lockHeading();
        pose = new Pose(0, 0, 1.0);
        drivetrain.setMecanumTargets(0, 0, 0, false);
        drivetrain.write();
    }

    @Test
    public void untilMeasuredAPressLeavesTheWheelsStill() throws Exception {
        measuredField().setBoolean(null, false);
        assertFalse(Drivetrain.headingLockAvailable());

        lockThenTurnOffTheLockedHeadingWithNoStick();

        assertFalse(drivetrain.isHeadingLocked());
        for (double power : drivetrain.getMotorPowers()) assertEquals(0, power, EPS);
        for (double power : wheelPowers.values()) assertEquals(0, power, EPS);
    }

    @Test
    public void untilMeasuredTheRightStickStillTurns() throws Exception {
        measuredField().setBoolean(null, false);
        drivetrain.lockHeading();
        drivetrain.setMecanumTargets(0, 0, 0.5, false);
        drivetrain.write();

        double[] powers = drivetrain.getMotorPowers();
        assertEquals(0.5, powers[0], EPS);
        assertEquals(0.5, powers[1], EPS);
        assertEquals(-0.5, powers[2], EPS);
        assertEquals(-0.5, powers[3], EPS);
    }

    @Test
    public void onceMeasuredTheLockTurnsTheChassisByItself() throws Exception {
        measuredField().setBoolean(null, true);
        assertTrue(Drivetrain.headingLockAvailable());

        lockThenTurnOffTheLockedHeadingWithNoStick();

        assertTrue(drivetrain.isHeadingLocked());
        double turn = 0;
        for (double power : drivetrain.getMotorPowers()) turn = Math.max(turn, Math.abs(power));
        assertTrue(String.valueOf(turn), turn > 0.5);
    }
}
