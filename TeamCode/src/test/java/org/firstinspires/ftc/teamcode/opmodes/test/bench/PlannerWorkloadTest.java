package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.architecture.planner.RouteOptimizer;
import org.firstinspires.ftc.teamcode.architecture.planner.VisibilityGraphPlanner;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

public class PlannerWorkloadTest {
    private final PlannerWorkload workload = new PlannerWorkload();
    private double sliceMs;

    @Before
    public void saveSlice() {
        sliceMs = VisibilityGraphPlanner.routeSearchSliceMs;
    }

    @After
    public void restoreSlice() {
        VisibilityGraphPlanner.routeSearchSliceMs = sliceMs;
    }

    private List<PlannerWorkload.Layout> layouts() {
        List<PlannerWorkload.Layout> out = new ArrayList<>();
        out.add(workload.bothSidesOfTheRail(4));
        for (PlannerWorkload.Layout l : workload.crafted()) {
            if (l.label.equals("5 balls behind the rail")) out.add(l);
        }
        assertEquals("the crafted layouts lost '5 balls behind the rail'", 2, out.size());
        return out;
    }

    private static final class Run {
        final List<Double> stepMs = new ArrayList<>();
        final List<Boolean> oneAdvanceEach = new ArrayList<>();
        RouteOptimizer.Route route;
    }

    private Run plan(PlannerWorkload.Layout layout, double slice, boolean watchAdvances) throws Exception {
        VisibilityGraphPlanner.routeSearchSliceMs = slice;
        RouteOptimizer.Search search = RouteOptimizer.search(layout.start, layout.balls, layout.start, workload.keepIn,
                workload.rails, PlannerWorkload.ROBOT, PlannerWorkload.RAIL_GAP_IN, false);
        Run run = new Run();
        boolean done = false;
        while (!done) {
            Snapshot before = watchAdvances ? new Snapshot(search) : null;
            long t0 = System.nanoTime();
            done = search.step();
            run.stepMs.add((System.nanoTime() - t0) / 1e6);
            if (watchAdvances) run.oneAdvanceEach.add(Snapshot.oneAdvance(before, new Snapshot(search)));
            assertTrue(layout.label + ": over 100000 steps", run.stepMs.size() < 100_000);
        }
        run.route = search.route();
        return run;
    }

    @Test
    public void infiniteSliceFinishesInOneStep() throws Exception {
        for (PlannerWorkload.Layout layout : layouts()) {
            Run run = plan(layout, Double.POSITIVE_INFINITY, false);
            assertEquals(layout.label + ": steps with an infinite slice", 1, run.stepMs.size());
            assertNotNull(run.route);
        }
    }

    @Test
    public void zeroSliceDoesOneAdvancePerStep() throws Exception {
        for (PlannerWorkload.Layout layout : layouts()) {
            Run run = plan(layout, 0, true);
            assertTrue(layout.label + ": a 0 ms slice should take many steps", run.stepMs.size() > 3);
            for (int i = 0; i < run.oneAdvanceEach.size(); i++) {
                assertTrue(layout.label + ": step " + (i + 1) + " of " + run.oneAdvanceEach.size()
                        + " didn't do exactly one advance()", run.oneAdvanceEach.get(i));
            }
        }
    }

    @Test
    public void twentyMsSliceYieldsBetweenSteps() throws Exception {
        PlannerWorkload.Layout slowest = layouts().get(1);
        Run whole = plan(slowest, Double.POSITIVE_INFINITY, false);
        Run sliced = plan(slowest, 20, false);
        Run single = plan(slowest, 0, false);
        assertTrue("the slowest layout should outlast one 20 ms slice (it took " + whole.stepMs.get(0) + " ms in one)",
                whole.stepMs.get(0) > 20);
        assertTrue("a 20 ms slice should take more than one step", sliced.stepMs.size() > 1);
        assertTrue("a 20 ms slice should take fewer steps than one advance each (" + sliced.stepMs.size() + " vs "
                + single.stepMs.size() + ")", sliced.stepMs.size() < single.stepMs.size());
        for (int i = 0; i < sliced.stepMs.size() - 1; i++) {
            assertTrue("step " + (i + 1) + " returned after " + sliced.stepMs.get(i) + " ms, before its 20 ms slice",
                    sliced.stepMs.get(i) >= 20);
        }
    }

    @Test
    public void slicingDoesNotChangeTheRoute() throws Exception {
        for (PlannerWorkload.Layout layout : layouts()) {
            RouteOptimizer.Route whole = plan(layout, Double.POSITIVE_INFINITY, false).route;
            for (double slice : new double[] {0, 20}) {
                RouteOptimizer.Route route = plan(layout, slice, false).route;
                assertArrayEquals(layout.label + ": order with a " + slice + " ms slice", whole.order, route.order);
                assertEquals(layout.label + ": length with a " + slice + " ms slice", whole.length, route.length, 1e-9);
            }
        }
    }

    @Test
    public void timingCarriesEverySlice() {
        VisibilityGraphPlanner.routeSearchSliceMs = 0;
        PlannerWorkload.Planning planning = workload.request(workload.bothSidesOfTheRail(4));
        Samples steps = new Samples();
        Samples overrun = new Samples();
        PlannerWorkload.Timing timing = null;
        while (timing == null) timing = planning.step(steps, overrun);
        assertEquals(timing.loops, timing.slicesMs.length);
        assertEquals(steps.count(), timing.slicesMs.length);
        double sum = 0;
        for (double ms : timing.slicesMs) sum += ms;
        assertEquals(timing.computeMs, sum, 1e-6);
        Map<?, ?> summary = (Map<?, ?>) PlannerWorkload.summarize(Collections.singletonList(timing)).get("stepMs");
        assertEquals(timing.slicesMs.length, summary.get("count"));
    }

    /** The Search's progress, read from its private fields: each advance() changes exactly one of them by one step. */
    private static final class Snapshot {
        final Object orders;
        final int next;
        final Object route;
        final Object plan;
        final int size;

        Snapshot(RouteOptimizer.Search search) throws Exception {
            orders = field(search, "orders");
            next = (Integer) field(search, "next");
            route = field(search, "route");
            plan = field(search, "plan");
            size = (Integer) field(search, "size");
        }

        private static Object field(Object o, String name) throws Exception {
            Field f = RouteOptimizer.Search.class.getDeclaredField(name);
            f.setAccessible(true);
            return f.get(o);
        }

        static boolean oneAdvance(Snapshot a, Snapshot b) {
            boolean sameSearch = b.orders == a.orders && b.size == a.size;
            if (a.plan == null && b.plan != null) {
                return a.route != null && b.route == a.route && sameSearch && b.next == a.next;
            }
            if (a.route == null && b.route != null) return b.plan == null && sameSearch && b.next == a.next;
            if (b.route != null || b.plan != null) return false;
            if (a.orders == null && b.orders != null) return b.next == 0 && b.size == a.size;
            if (a.orders != null && sameSearch) return b.next == a.next + 1;
            return a.orders != null && b.orders == null && b.size == a.size - 1;
        }
    }
}
