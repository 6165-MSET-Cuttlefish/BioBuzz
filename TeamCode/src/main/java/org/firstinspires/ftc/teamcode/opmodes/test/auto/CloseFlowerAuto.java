package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.architecture.command.PathCommands;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzOpMode;
import org.firstinspires.ftc.teamcode.modules.Camera;

/** CloseFlower on the framework; {@link CloseFlowerLinearAuto} drives the same paths without it, for comparison. */
@Autonomous(name = "Close Flower Auto", group = "Test")
public class CloseFlowerAuto extends BioBuzzOpMode {

    private static final double ROUTINE_TIMEOUT_MS = 30000;

    private Command routine;

    @Override
    protected void initialize() {
        // Pedro drives fl/bl/fr/br through its own motor objects; Drivetrain.write() would fight it for them.
        robot.drivetrain.setWriteEnabled(false);
        // A path test; disabled, the Camera never polls the Limelight or needs the ball script on it.
        if (!Camera.VisionState.DISABLED.activate()) throw new IllegalStateException("Camera didn't take DISABLED");

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
