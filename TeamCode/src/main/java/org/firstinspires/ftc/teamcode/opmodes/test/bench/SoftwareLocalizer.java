package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;

/** A Localizer with no hardware: the pose stays wherever setPose() put it, so a bare hub can run the follower. */
final class SoftwareLocalizer implements Localizer {
    private MotionState state = MotionState.zero();

    @Override
    public void setPose(Pose pose) {
        state = state.withPose(pose);
    }

    @Override
    public MotionState state() {
        return state;
    }

    @Override
    public void update() {}

    @Override
    public void reset() {
        state = MotionState.zero();
    }
}
