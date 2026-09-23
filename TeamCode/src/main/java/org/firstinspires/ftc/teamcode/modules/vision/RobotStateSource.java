package org.firstinspires.ftc.teamcode.modules.vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

public interface RobotStateSource {

    RobotStateHistory.Sample sample(double timestampSeconds);

    static RobotStateSource fromFollower(final Follower follower) {
        return timestampSeconds -> {
            Pose pose = follower.pose();
            Velocity velocity = follower.velocity();
            return new RobotStateHistory.Sample(
                    pose.x(), pose.y(), pose.heading(),
                    velocity.vx, velocity.vy, velocity.omega, timestampSeconds);
        };
    }
}
