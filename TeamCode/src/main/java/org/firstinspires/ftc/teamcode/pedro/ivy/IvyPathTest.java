package org.firstinspires.ftc.teamcode.pedro.ivy;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.architecture.command.PathCommands;
import org.firstinspires.ftc.teamcode.architecture.command.StateCommands;
import org.firstinspires.ftc.teamcode.decode.DecodeOpMode;
import org.firstinspires.ftc.teamcode.decode.modules.Shooter;

@Autonomous(name = "Ivy Path Test", group = "Test")
public class IvyPathTest extends DecodeOpMode {

    private static final double ROUTINE_TIMEOUT_MS = 10000;

    private final Pose start = new Pose(48, 96, 0);
    private final Pose end = new Pose(96, 48, 0);

    private Command routine;

    @Override
    protected void initialize() {
        robot.drivetrain.setWriteEnabled(false);
        robot.follower.setPose(start);

        Path spline = curve(start, new Pose(72, 96), new Pose(96, 72), end).tangent();

        routine = PathCommands.timeout(parallel(
                PathCommands.follow(robot.follower, spline),
                sequential(
                        waitUntil(() -> robot.follower.completion() >= 0.5),
                        StateCommands.set(Shooter.FlywheelState.POWER))
        ), ROUTINE_TIMEOUT_MS);
    }

    // DecodeOpMode re-applies its dashboard write toggles every loop, which would re-enable drivetrain writes.
    @Override
    protected void onLoopStart() {
        super.onLoopStart();
        robot.drivetrain.setWriteEnabled(false);
    }

    @Override
    protected void onStart() {
        Scheduler.schedule(routine);
    }

    @Override
    protected void telemetry() {
        telemetry.addData("Running", Scheduler.isRunning(routine));
        telemetry.addData("Path Completion", "%.2f", robot.follower.completion());
        telemetry.addData("Flywheel State", robot.shooter.getState(Shooter.FlywheelState.class));
    }
}
