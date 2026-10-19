package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.architecture.command.PathCommands;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzOpMode;

/** CloseFlower on the framework; {@link CloseFlowerLinearAuto} drives the same paths without it, for comparison. */
@Autonomous(name = "Close Flower Auto", group = "Test")
public class CloseFlowerAuto extends BioBuzzOpMode {

    private static final double ROUTINE_TIMEOUT_MS = 30000;

    private Command routine;

    @Override
    protected void initialize() {
        robot.drivetrain.setWriteEnabled(false);
        disableBallVision();

        CloseFlowerPaths paths = new CloseFlowerPaths();
        robot.follower.setPose(paths.start);

        Path[] legs = paths.all();
        Command[] follows = new Command[legs.length];
        for (int i = 0; i < legs.length; i++) follows[i] = PathCommands.follow(robot.follower, legs[i]);
        routine = PathCommands.timeout(sequential(follows), ROUTINE_TIMEOUT_MS);
    }

    @Override
    protected void onStart() {
        Scheduler.schedule(routine);
    }

    @Override
    protected void telemetry() {
        telemetry.addData("Routine Running", Scheduler.isRunning(routine));
        if (robot.follower.currentPath() != null) {
            telemetry.addData("Path Index", robot.follower.pathIndex());
            telemetry.addData("Distance Remaining", "%.1f in", robot.follower.distanceToEndpoint());
        }
    }
}
