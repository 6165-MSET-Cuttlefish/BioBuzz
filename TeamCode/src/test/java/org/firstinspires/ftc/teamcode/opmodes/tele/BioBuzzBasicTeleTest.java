package org.firstinspires.ftc.teamcode.opmodes.tele;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class BioBuzzBasicTeleTest {
    private static final double EPS = 1e-9;

    @Test
    public void forwardDrivesAllFourWheelsForwardAtTheLimit() {
        assertArrayEquals(new double[] {0.6, 0.6, 0.6, 0.6}, BioBuzzBasicTele.mecanum(1, 0, 0, 0.6), EPS);
    }

    @Test
    public void strafeRightSpinsTheDiagonalsAgainstEachOther() {
        assertArrayEquals(new double[] {0.6, -0.6, -0.6, 0.6}, BioBuzzBasicTele.mecanum(0, 1, 0, 0.6), EPS);
    }

    @Test
    public void turnRightDrivesTheLeftSideForwardAndTheRightSideBack() {
        assertArrayEquals(new double[] {0.6, 0.6, -0.6, -0.6}, BioBuzzBasicTele.mecanum(0, 0, 1, 0.6), EPS);
    }

    @Test
    public void noWheelExceedsTheLimitWhenEveryStickIsFull() {
        for (double wheel : BioBuzzBasicTele.mecanum(1, 1, 1, 0.6)) assertEquals(true, Math.abs(wheel) <= 0.6 + EPS);
        assertEquals(0.6, BioBuzzBasicTele.mecanum(1, 1, 1, 0.6)[0], EPS);
    }

    @Test
    public void aSmallStickGivesAProportionallySmallPower() {
        assertArrayEquals(new double[] {0.15, 0.15, 0.15, 0.15}, BioBuzzBasicTele.mecanum(0.25, 0, 0, 0.6), EPS);
    }

    @Test
    public void deadbandZeroesOnlyValuesInsideIt() {
        assertEquals(0, BioBuzzBasicTele.deadband(0.04, 0.05), EPS);
        assertEquals(-0.06, BioBuzzBasicTele.deadband(-0.06, 0.05), EPS);
    }

    @Test
    public void intakeRunsInOnTheRightTriggerOutOnTheLeftAndStopsWithNeither() {
        assertEquals(1.0, BioBuzzBasicTele.intakePower(0.5, 0), EPS);
        assertEquals(-1.0, BioBuzzBasicTele.intakePower(0, 0.5), EPS);
        assertEquals(0, BioBuzzBasicTele.intakePower(0.05, 0.05), EPS);
        assertEquals(1.0, BioBuzzBasicTele.intakePower(1, 1), EPS);
    }

    @Test
    public void flywheelRunsOpenLoopUntilATargetRpmIsSet() {
        BioBuzzBasicTele.Flywheel flywheel = new BioBuzzBasicTele.Flywheel();
        flywheel.power = 0.6;
        assertEquals(0.6, BioBuzzBasicTele.flywheelPower(flywheel, 0), EPS);
        flywheel.targetRpm = 3000;
        flywheel.kV = 0.0002;
        flywheel.kP = 0.001;
        assertEquals(0.7, BioBuzzBasicTele.flywheelPower(flywheel, 2900), EPS);
        assertEquals(0, BioBuzzBasicTele.flywheelPower(flywheel, 4000), EPS);
    }
}
