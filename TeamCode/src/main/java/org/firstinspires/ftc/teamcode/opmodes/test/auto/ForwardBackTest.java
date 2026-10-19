package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.groups.Groups.sequential;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.architecture.command.PathCommands;

@Autonomous(name = "ForwardBackTest", group = "Test")
public class ForwardBackTest extends BarePedroAuto {

    private static final double ROUTINE_TIMEOUT_MS = 30000;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.5205, 19.2527, 90);
    private final Pose forward = poseFactory.of(56.5205, 99.2527, 90);

    @Override
    protected Pose startPose() {
        return start;
    }

    @Override
    protected Command routine() {
        return PathCommands.timeout(sequential(
                PathCommands.follow(follower, forwardPath()),
                PathCommands.follow(follower, backPath())
        ), ROUTINE_TIMEOUT_MS);
    }

    private Path forwardPath() {
        return line(start, forward).constant(start.heading());
    }

    private Path backPath() {
        return line(forward, start).constant(start.heading());
    }
}
