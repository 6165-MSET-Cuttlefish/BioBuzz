package org.firstinspires.ftc.teamcode.pedro.ivy;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.concurrent.atomic.AtomicBoolean;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization;
import org.firstinspires.ftc.teamcode.architecture.auto.PathHeadings;
import org.firstinspires.ftc.teamcode.architecture.command.PathCommands;
import org.firstinspires.ftc.teamcode.architecture.command.StateCommands;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.modules.BettaShooter;
import org.firstinspires.ftc.teamcode.modules.BettaTurret;
import org.firstinspires.ftc.teamcode.opmodes.test.auto.BallCollectionRobot;

@Autonomous(name = "Ivy Path Test", group = "Test")
public class IvyPathTest extends EnhancedOpMode {

    private static final double ROUTINE_TIMEOUT_MS = 10 * 60 * 1000;
    private static final double PAUSE_MS = 2000;

    private final Pose start = new Pose(48, 48, 0);
    private final Pose control1 = new Pose(72, 48);
    private final Pose control2 = new Pose(96, 72);
    private final Pose end = new Pose(96, 96);

    private BettaTurret turret;
    private BettaShooter shooter;
    private Command routine;
    private double[][] splineDrawing;
    private String leg = "waiting for START";
    private double halfwayFiredAt = Double.NaN;



    @Override
    protected Robot createRobot() {
        turret = new BettaTurret(hardwareMap);
        shooter = new BettaShooter(hardwareMap);

        return new BallCollectionRobot(this);
    }

    @Override
    protected void initialize() {
        robot.follower.setPose(start);

        Path forward = curve(start, control1, control2, end).tangent();
        Path back = PathHeadings.backwardTangent(curve(end, control2, control1, start));
        splineDrawing = FieldVisualization.pathPolyline(forward);

        Command aimSideways = StateCommands.set(BettaTurret.RightState.SIDEWAYS);
        Command aimForward = StateCommands.set(BettaTurret.RightState.FORWARD);
        Command startShooter = StateCommands.set(BettaShooter.ShooterState.ON);
        Command stopShooter = StateCommands.set(BettaShooter.ShooterState.OFF);

        routine = sequential(PathCommands.timeout(Groups.loop(sequential(
                instant(() -> leg = "forward"),
                parallel(PathCommands.follow(robot.follower, forward), sequential(halfway(), recordHalfway(), aimSideways, startShooter)),
                instant(() -> leg = "pause"),
                PathCommands.waitMs(PAUSE_MS),
                instant(() -> leg = "back"),
                parallel(PathCommands.follow(robot.follower, back), sequential(halfway(), recordHalfway(), aimForward, stopShooter)),
                instant(() -> leg = "pause"),
                PathCommands.waitMs(PAUSE_MS)
        )), ROUTINE_TIMEOUT_MS),
                instant(() -> leg = "timed out"));
    }

    // completion() reads 1 until Pedro updates on the new path, so wait to see it under half first.
    private Command halfway() {
        AtomicBoolean armed = new AtomicBoolean();
        return Command.build()
                .setStart(() -> armed.set(false))
                .setExecute(() -> {
                    if (robot.follower.completion() < 0.5) armed.set(true);
                })
                .setDone(() -> armed.get() && robot.follower.completion() >= 0.5);
    }

    private Command recordHalfway() {
        return instant(() -> halfwayFiredAt = robot.follower.completion());
    }

    @Override
    protected void onStart() {
        Scheduler.schedule(routine);
    }

    @Override
    protected void telemetry() {
        telemetry.addData("Running", Scheduler.isRunning(routine));
        telemetry.addData("Leg", leg);
        telemetry.addData("Path Completion", "%.2f", robot.follower.completion());
        telemetry.addData("Halfway Fired At Completion", "%.2f", halfwayFiredAt);
        telemetry.addData("Turret Right State", turret.getState(BettaTurret.RightState.class));
    }

    @Override
    protected void dashboardOverlay(Canvas overlay) {
        overlay.setStroke("#2962FF");
        overlay.strokePolyline(splineDrawing[0], splineDrawing[1]);
    }
}
