package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * EnhancedOpMode's failure path: the runner's "throwIn" param picks the hook that throws (initialize,
 * initializeLoop, onStart, gameLoop, moduleInit, or none). Three stub modules log every initStates, init and stop
 * to failure_&lt;throwIn&gt;.json as it happens; stub B's stop() always throws, so a thrown hook's exception
 * should carry it as Suppressed, and with "none" stop() itself should rethrow it.
 */
@TeleOp(name = "Bench: Failure Path", group = "Test")
public class FailurePathBench extends EnhancedOpMode {
    static final String INJECTED = "BENCH injected failure in ";
    static final String STUB_B_STOP = "BENCH stub B stop() failure, expected";

    public static final class StubModule extends Module {
        private final String label;
        private final FailurePathBench bench;

        StubModule(String label, FailurePathBench bench) {
            this.label = label;
            this.bench = bench;
        }

        @Override
        protected void initStates() {
            bench.event("initStates " + label);
        }

        @Override
        public void init() {
            bench.event("init " + label);
            if (label.equals("B") && bench.throwIn.equals("moduleInit")) throw new IllegalStateException(INJECTED + "module B init()");
        }

        @Override protected void read() {}
        @Override protected void write() {}

        @Override
        public void stop() {
            bench.event("stop " + label);
            if (label.equals("B")) throw new IllegalStateException(STUB_B_STOP);
        }
    }

    public static class FailureRobot extends Robot {
        public StubModule stubA;
        public StubModule stubB;
        public StubModule stubC;

        public FailureRobot(EnhancedOpMode opMode) {
            super(opMode);
        }

        @Override
        protected Follower createFollower(HardwareMap hardwareMap) {
            return BenchRobot.softwareFollower(hardwareMap);
        }

        @Override
        protected void initializeGameModules() {
            FailurePathBench bench = (FailurePathBench) opMode;
            stubA = new StubModule("A", bench);
            stubB = new StubModule("B", bench);
            stubC = new StubModule("C", bench);
        }
    }

    private String throwIn;
    private final List<String> events = new ArrayList<>();
    private final Map<String, Object> results = new LinkedHashMap<>();
    private int initLoops;
    private int loops;

    @Override
    protected Robot createRobot() {
        throwIn = BenchIO.param(BenchIO.params(), "throwIn", "none");
        results.put("throwIn", throwIn);
        results.put("events", events);
        event("createRobot");
        return new FailureRobot(this);
    }

    void event(String what) {
        events.add(what);
        BenchIO.log("failure path event: %s", what);
        BenchIO.write("failure_" + throwIn, results);
    }

    private void maybeThrow(String hook) {
        if (throwIn.equals(hook)) {
            event("throwing in " + hook);
            throw new IllegalStateException(INJECTED + hook);
        }
    }

    @Override
    protected void initialize() {
        event("initialize");
        maybeThrow("initialize");
    }

    @Override
    protected void initializeLoop() {
        if (++initLoops == 3) maybeThrow("initializeLoop");
    }

    @Override
    protected void onStart() {
        event("onStart");
        maybeThrow("onStart");
    }

    @Override
    protected void gameLoop() {
        if (++loops == 3) maybeThrow("gameLoop");
    }

    @Override
    protected void telemetry() {
        telemetry.addData(BenchReport.KEY_PHASE, "failure path " + throwIn);
        telemetry.addData("Bench loops", loops);
        // Only "none" gets here: it is done once running, and the runner's STOP is the test.
        if (loops >= 10) telemetry.addData(BenchReport.KEY_DONE, "running; STOP now to exercise stop()");
    }

    @Override
    protected void onEnd() {
        event("onEnd");
    }
}
