package org.firstinspires.ftc.teamcode.pedro;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;

import org.junit.Before;
import org.junit.Test;

import java.util.Collections;
import java.util.Map;

/** BioBuzz Tele's position lock relies on Pedro's hold pushing back for as long as it runs, not only until it settles. */
public class PedroHoldTest {
    private static final Pose LOCK = new Pose(50, 50, 0);

    private final StillLocalizer localizer = new StillLocalizer();
    private final RecordingDrivetrain drivetrain = new RecordingDrivetrain();
    private Follower follower;

    @Before
    public void settle() throws InterruptedException {
        follower = new Follower(localizer, drivetrain, BioBuzzConstants.foresight());
        localizer.setPose(LOCK);
        follower.hold(LOCK);
        follower.update();
        // Past ForesightConfig's 100 ms timeoutConstraint, after which Pedro reports the hold settled.
        Thread.sleep(150);
        follower.update();
        assertFalse("settled", follower.isBusy());
    }

    @Test
    public void shovedForwardDrivesBack() {
        DrivePowers p = shoveTo(new Pose(53, 50, 0));
        assertTrue(String.valueOf(p), p.forward() < 0);
    }

    @Test
    public void shovedEitherWaySidewaysStrafesBackTheOtherWay() {
        double left = shoveTo(new Pose(50, 53, 0)).strafe();
        double right = shoveTo(new Pose(50, 47, 0)).strafe();
        assertTrue(left + " then " + right, left != 0 && Math.signum(left) == -Math.signum(right));
    }

    @Test
    public void turnedCounterclockwiseTurnsBack() {
        DrivePowers p = shoveTo(new Pose(50, 50, 0.3));
        assertTrue(String.valueOf(p), p.turn() < 0);
    }

    @Test
    public void stillHoldingAfterRepeatedShoves() {
        for (int i = 0; i < 20; i++) shoveTo(new Pose(50 + (i % 2 == 0 ? 2 : -2), 50, 0));
        assertTrue(follower.holding());
        assertTrue(String.valueOf(drivetrain.last), shoveTo(new Pose(53, 50, 0)).forward() < 0);
    }

    private DrivePowers shoveTo(Pose pose) {
        localizer.setPose(pose);
        follower.update();
        return drivetrain.last;
    }

    private static final class StillLocalizer implements Localizer {
        private Pose pose = new Pose(0, 0, 0);

        @Override
        public void setPose(Pose pose) {
            this.pose = pose;
        }

        @Override
        public MotionState state() {
            return MotionState.ofTwist(pose, Twist.zero());
        }

        @Override
        public void update() {}

        @Override
        public void reset() {}
    }

    private static final class RecordingDrivetrain implements Drivetrain {
        DrivePowers last = DrivePowers.zero();

        @Override
        public void drive(DrivePowers powers, boolean manual) {
            last = powers;
        }

        @Override
        public double maxScaling(DrivePowers a, DrivePowers b) {
            return 1;
        }

        @Override
        public void stop() {
            last = DrivePowers.zero();
        }

        @Override
        public void stop(boolean brake) {
            stop();
        }

        @Override
        public Map<String, Object> debug() {
            return Collections.emptyMap();
        }

        @Override
        public double interpolateVelocity(double a, double b, double c) {
            return 0;
        }
    }
}
