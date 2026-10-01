package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class CloseFlowerTesting extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose closeFlower = poseFactory.of(14.9769, 46.9989, 179.316);
    private final Pose closeFlowerControl1 = poseFactory.of(56, 23, 0);
    private final Pose closeFlowerControl2 = poseFactory.of(34.9769, 46.9989, 0);
    private final Pose path2turnStart = poseFactory.of(14.9769, 46.9989, 180);
    private final Pose path2turn = poseFactory.of(29.0321, 76.6058, 65.7039);
    private final Pose path2turnControl1 = poseFactory.of(23.0138, 63.2743, 0);
    private final Pose path2 = poseFactory.of(46.9748, 130.0998, 90.091);
    private final Pose path2Control1 = poseFactory.of(47.087, 116.7758, 0);
    private final Pose point4 = poseFactory.of(57.0019, 101.165, 270);
    private final Pose point4Segment1Heading = poseFactory.of(57.0019, 101.165, 90.5);
    private final Pose point4Segment2Start = poseFactory.of(57.0019, 101.165, 90.5);
    private final Pose point4Segment2End = poseFactory.of(57.0019, 101.165, 270);
    private final Pose point5 = poseFactory.of(45.3325, 8.4566, -176.9646);
    private final Pose point5Control1 = poseFactory.of(61.5713, 0.6836, 0);
    private final Pose point5Control2 = poseFactory.of(77.8952, 10.2674, 0);
    private final Pose point6 = poseFactory.of(8.3412, 8.3306, -179.8049);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, closeFlower()),
                follow(follower, path2turn()),
                follow(follower, path2()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = BettaConstants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path closeFlower() {
        return Paths.curve(start, closeFlowerControl1, closeFlowerControl2, closeFlower).tangent();
    }

    public Path path2turn() {
        return Paths.curve(path2turnStart, path2turnControl1, path2turn).linear(path2turnStart, path2turn);
    }

    public Path path2() {
        return Paths.curve(path2turn, path2Control1, path2).tangent();
    }

    public Path path4() {
        return Paths.line(path2, point4).heading(Interpolator.piecewise().until(0.5, Interpolator.constant(point4Segment1Heading)).until(1, Interpolator.linear(point4Segment2Start, point4Segment2End)));
    }

    public Path path5() {
        return Paths.curve(point4, point5Control1, point5Control2, point5).tangent();
    }

    public Path path6() {
        return Paths.line(point5, point6).tangent();
    }
}
