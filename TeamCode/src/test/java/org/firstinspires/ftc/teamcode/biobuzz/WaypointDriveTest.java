package org.firstinspires.ftc.teamcode.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.planner.Ball;
import org.firstinspires.ftc.teamcode.architecture.planner.Obstacle;
import org.firstinspires.ftc.teamcode.architecture.planner.Region;
import org.firstinspires.ftc.teamcode.architecture.planner.RouteOptimizer;
import org.firstinspires.ftc.teamcode.architecture.planner.RoutePathBuilder;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.Collections;

public class WaypointDriveTest {
    private final WaypointDrive.Settings settings = new WaypointDrive.Settings();
    private AllianceColor alliance;

    @Before
    public void field() {
        alliance = Context.allianceColor;
        Context.allianceColor = AllianceColor.RED;
        FieldConfig.setSymmetry(BioBuzzField.SYMMETRY);
    }

    @After
    public void restore() {
        Context.allianceColor = alliance;
        FieldConfig.clear();
    }

    private static RouteOptimizer.Search finish(RouteOptimizer.Search search) {
        for (int i = 0; i < 10_000; i++) {
            if (search.step()) return search;
        }
        throw new AssertionError("the search didn't finish in 10000 steps");
    }

    @Test
    public void openDriveIsOneLineTurningOnTheWay() {
        Pose start = new Pose(20, 20, 0);
        Pose target = new Pose(40, 30, Math.toRadians(90));
        RouteOptimizer.Search search = finish(WaypointDrive.search(start, target, settings));
        assertNull(search.noRoute());
        RoutePathBuilder.Plan plan = search.plan();
        assertNull("no balls, so no intake leg", plan.intake);
        assertNotNull(plan.back);
        assertEquals(RoutePathBuilder.Reroute.NONE, plan.returnReroute);
        assertEquals("one drive, no turn in place", 1, plan.steps.size());
        assertNotNull(plan.steps.get(0).path);
    }

    @Test
    public void driveAcrossOurRailGoesAroundItsEnd() {
        Pose start = new Pose(30, 71, 0);
        Pose target = new Pose(60, 105, 0);
        assertNull(WaypointDrive.poseProblem(start, settings));
        assertNull(WaypointDrive.poseProblem(target, settings));
        RouteOptimizer.Search search = finish(WaypointDrive.search(start, target, settings));
        assertNull(search.noRoute());
        RoutePathBuilder.Plan plan = search.plan();
        assertTrue("the straight line crosses the rail, so the drive is rerouted",
                plan.returnReroute != RoutePathBuilder.Reroute.NONE);
        double straight = Math.hypot(30, 34);
        assertTrue("a detour is longer than the straight line: " + plan.returnLength, plan.returnLength > straight + 2);
    }

    @Test
    public void turningAcrossTheRailToBesideTheHiveIsOneSpline() {
        Pose start = new Pose(20, 20, 0);
        Pose target = new Pose(58, 110, Math.toRadians(180));
        RouteOptimizer.Search search = finish(WaypointDrive.search(start, target, settings));
        assertNull(search.noRoute());
        RoutePathBuilder.Plan plan = search.plan();
        assertEquals(RoutePathBuilder.Reroute.SPLINE, plan.returnReroute);
        assertEquals("turning on the way, never stopping to turn", 1, plan.steps.size());
    }

    @Test
    public void alreadyThereDrivesNothing() {
        Pose here = new Pose(20, 20, 0);
        RoutePathBuilder.Plan plan = finish(WaypointDrive.search(here, here, settings)).plan();
        assertTrue(plan.steps.isEmpty());
    }

    @Test
    public void poseProblemKeepsToOurHalfAndOffTheRails() {
        assertNull(WaypointDrive.poseProblem(new Pose(20, 20, 0), settings));
        assertNotNull("BLUE's half", WaypointDrive.poseProblem(new Pose(100, 30, 0), settings));
        assertNotNull("on RED's rail", WaypointDrive.poseProblem(new Pose(47, 71, 0), settings));
        assertNotNull("touching the wall", WaypointDrive.poseProblem(new Pose(5, 20, 0), settings));
        String nan = WaypointDrive.poseProblem(new Pose(Double.NaN, 20, 0), settings);
        assertTrue(nan, nan != null && nan.contains("finite"));
    }

    @Test
    public void blueKeepsToBluesHalf() {
        Context.allianceColor = AllianceColor.BLUE;
        double w = FieldConfig.fieldWidthInches;
        assertNull(WaypointDrive.poseProblem(new Pose(w - 20, w - 20, Math.PI), settings));
        assertNotNull(WaypointDrive.poseProblem(new Pose(20, 20, 0), settings));
    }

    @Test
    public void unreachableReturnPoseIsReportedNotThrown() {
        Region area = new Region(0, 100, 0, 40);
        Obstacle wall = new Obstacle(45, 55, -10, 50);
        RouteOptimizer.Search search = finish(RouteOptimizer.search(new Pose(20, 20, 0), new Ball[0],
                new Pose(80, 20, 0), area, Collections.singletonList(wall), RobotGeometry.shape(), 1, false));
        assertNotNull(search.noRoute());
        try {
            search.plan();
            fail("plan() returned with no route");
        } catch (IllegalStateException expected) {
            assertEquals(search.noRoute(), expected.getMessage());
        }
    }
}
