package org.firstinspires.ftc.teamcode.decode;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.PathSegment;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.architecture.command.StateCommands;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.decode.modules.Endgame;
import org.firstinspires.ftc.teamcode.decode.modules.Magazine;
import org.firstinspires.ftc.teamcode.decode.modules.Shooter;

import java.util.function.BooleanSupplier;

public class DecodeActions {
    private final DecodeRobot robot;

    private Command endgameCommand;

    public DecodeActions(DecodeRobot robot) {
        this.robot = robot;
    }

    private Magazine.HorizontalBackState backShootState() {
        Magazine.HorizontalBackState state = robot.magazine.getState(Magazine.HorizontalBackState.class);
        if (state == Magazine.HorizontalBackState.OPEN) return Magazine.HorizontalBackState.OPEN_SHOOT;
        if (state == Magazine.HorizontalBackState.STORED) return Magazine.HorizontalBackState.STORED_SHOOT;
        return state;
    }

    private Magazine.HorizontalBackState backResetState() {
        Magazine.HorizontalBackState state = robot.magazine.getState(Magazine.HorizontalBackState.class);
        if (state == Magazine.HorizontalBackState.OPEN_SHOOT) return Magazine.HorizontalBackState.OPEN;
        if (state == Magazine.HorizontalBackState.STORED_SHOOT) return Magazine.HorizontalBackState.STORED;
        return state;
    }

    // Ivy spends a loop per sequential child, so steps that must land in the same tick share one command.
    private static CommandBuilder instantOn(Module module, Runnable body) {
        return instant(body).requiring(module);
    }

    public CommandBuilder prepareShooterFar() {
        return StateCommands.set(Shooter.FlywheelState.FAR_AUTO, Shooter.HoodState.FAR_AUTO);
    }

    public CommandBuilder prepareShooterClose() {
        return StateCommands.set(Shooter.FlywheelState.PID);
    }

    public CommandBuilder intakeUntilFull() {
        return sequential(
                StateCommands.set(Magazine.IntakeState.FORWARD, Magazine.VerticalState.HALF_DOWN),
                waitUntil(() -> robot.magazine.intakeIsFull),
                StateCommands.set(Magazine.IntakeState.IDLE));
    }

    public CommandBuilder intakeOn() {
        return StateCommands.set(Magazine.IntakeState.FORWARD);
    }

    public CommandBuilder shootWhenReady() {
        return sequential(
                waitUntil(() -> robot.shooter.isAtTargetVelocity()),
                StateCommands.set(Magazine.VerticalState.ON));
    }

    public CommandBuilder shootAll(boolean resetAfterShoot, boolean firstBallDelay) {
        return sequential(
                instantOn(robot.magazine, () -> {
                    Magazine.IntakeState.OFF.activate();
                    Magazine.VerticalState.ON.activate();
                    backShootState().activate();
                }),
                waitMs(100 + (firstBallDelay ? 400 : 0)),
                StateCommands.set(Magazine.IntakeState.SHOOTING),
                waitMs(firstBallDelay ? 1100 : 1000),
                instantOn(robot.magazine, () -> {
                    if (resetAfterShoot) {
                        Magazine.IntakeState.FORWARD.activate();
                        Magazine.VerticalState.HALF_DOWN.activate();
                        backResetState().activate();
                    }
                }));
    }

    public CommandBuilder shootSorted(boolean resetAfterShoot) {
        return conditional(
                () -> robot.magazine.getState(Magazine.HorizontalFrontState.class) == Magazine.HorizontalFrontState.OPEN
                        && robot.magazine.getState(Magazine.HorizontalBackState.class) == Magazine.HorizontalBackState.OPEN,
                shootAll(resetAfterShoot, true),
                sequential(
                        instantOn(robot.magazine, () -> {
                            robot.magazine.shotSorted = false;
                            Magazine.IntakeState.OFF.activate();
                            Magazine.VerticalState.ON.activate();
                            backShootState().activate();
                        }),
                        waitMs(100),
                        StateCommands.set(Magazine.IntakeState.SHOOTING),
                        waitMs(300),
                        StateCommands.set(Magazine.HorizontalFrontState.OPEN, Magazine.HorizontalBackState.OPEN_SHOOT, Magazine.IntakeState.OFF),
                        waitMs(100),
                        StateCommands.set(Magazine.IntakeState.SHOOTING),
                        waitMs(800),
                        instantOn(robot.magazine, () -> {
                            if (resetAfterShoot) {
                                Magazine.IntakeState.IDLE.activate();
                                Magazine.VerticalState.HALF_DOWN.activate();
                                Magazine.HorizontalBackState.OPEN.activate();
                            }
                            robot.magazine.shotSorted = true;
                        })));
    }

    public CommandBuilder intakeOffThenOn(long delay) {
        return sequential(
                StateCommands.set(Magazine.IntakeState.OFF),
                waitMs(delay),
                StateCommands.set(Magazine.IntakeState.FORWARD));
    }

    public CommandBuilder flickExtakeThenOn(long delay) {
        return sequential(
                StateCommands.set(Magazine.IntakeState.AUTO_EXTAKE),
                waitMs(delay),
                StateCommands.set(Magazine.IntakeState.FORWARD));
    }

    public CommandBuilder flickExtakeThenOffThenOn(long delay1, long delay2) {
        return sequential(
                waitMs(50),
                StateCommands.set(Magazine.IntakeState.AUTO_EXTAKE),
                waitMs(delay1),
                StateCommands.set(Magazine.IntakeState.OFF),
                waitMs(delay2),
                StateCommands.set(Magazine.IntakeState.FORWARD));
    }

    public CommandBuilder shootAllPreloadMoving() {
        return sequential(
                waitUntil(() -> robot.shooter.isWithinLUTRange()),
                StateCommands.set(Magazine.IntakeState.IDLE, Magazine.VerticalState.ON, Magazine.HorizontalBackState.OPEN_SHOOT),
                waitMs(600),
                StateCommands.set(Magazine.HorizontalFrontState.OPEN_SHOOT),
                waitMs(300),
                StateCommands.set(Magazine.IntakeState.FORWARD, Magazine.VerticalState.OFF,
                        Magazine.HorizontalBackState.OPEN, Magazine.HorizontalFrontState.OPEN));
    }

    public CommandBuilder sortMagazine() {
        final boolean[] sorted = {false};
        return sequential(
                instantOn(robot.magazine, () -> {
                    Magazine.IntakeState.IDLE.activate();
                    Magazine.StorageDecision decision =
                            robot.magazine.checkAndStoreBalls(DecodeContext.motif);

                    sorted[0] = decision.storeInBack || decision.storeInFront;
                    if (!sorted[0]) return;

                    (decision.storeInBack
                            ? Magazine.HorizontalBackState.STORED
                            : Magazine.HorizontalBackState.OPEN).activate();
                    (decision.storeInFront
                            ? Magazine.HorizontalFrontState.STORED
                            : Magazine.HorizontalFrontState.OPEN).activate();
                }),
                conditional(() -> sorted[0],
                        sequential(waitMs(400), StateCommands.set(Magazine.IntakeState.FORWARD)),
                        instant(() -> {})));
    }

    public CommandBuilder endgameSequence() {
        CommandBuilder command = sequential(
                StateCommands.set(Endgame.InitialState.LIFT),
                waitUntil(() -> robot.endgame.initialLiftComplete()),
                // FULL_LIFT targets fl/fr encoder counts from zero, so reset them right before the PTOs engage.
                instantOn(robot.endgame, () -> {
                    robot.drivetrain.getFl().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.drivetrain.getFr().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.drivetrain.getFl().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.drivetrain.getFr().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Endgame.LeftPtoState.DOWN.activate();
                    Endgame.RightPtoState.DOWN.activate();
                }),
                waitMs(300),
                StateCommands.set(Endgame.InitialState.DISABLED, Endgame.FullLiftState.FULL_LIFT));
        endgameCommand = command;
        return command;
    }

    public CommandBuilder cancelEndgameSequence() {
        return instantOn(robot.endgame, () -> {
            if (endgameCommand != null) endgameCommand.cancel();
            Endgame.InitialState.DISABLED.activate();
            Endgame.FullLiftState.OFF.activate();
            robot.drivetrain.setRawTargets(0, 0, 0, 0);
        });
    }

    public CommandBuilder wiggleFrontHzPusher() {
        return sequential(
                StateCommands.set(Magazine.HorizontalFrontState.OPEN),
                waitMs(125),
                StateCommands.set(Magazine.HorizontalFrontState.OPEN_SHOOT),
                waitMs(125),
                StateCommands.set(Magazine.HorizontalFrontState.OPEN));
    }

    public CommandBuilder wiggleBackHzPusher() {
        return sequential(
                StateCommands.set(Magazine.HorizontalBackState.OPEN),
                waitMs(125),
                StateCommands.set(Magazine.HorizontalBackState.OPEN_SHOOT));
    }

    // No requirement: it only reads the Limelight, and a turret command must not end it.
    public CommandBuilder detectObeliskLoop() {
        return Command.build()
                .setStart(() -> robot.turret.detectingObelisk = true)
                .setExecute(() -> {
                    if (robot.turret.detectingObelisk) robot.turret.detectObelisk();
                })
                .setDone(() -> !robot.turret.detectingObelisk);
    }

    public CommandBuilder detectObelisk() {
        return instant(() -> robot.turret.detectObelisk());
    }

    public CommandBuilder autoLockSOTMDecel(Pose target, double velocity) {
        return lockWhile(() -> distanceRemainingOnSegment() > 0, () -> {
            double heading = target.heading();
            double distRemaining = distanceRemainingOnSegment();

            double currentVelocity = velocity * Math.min(1.0, distRemaining / 8.0);

            double predictedX = target.x() + Math.cos(heading) * currentVelocity * robot.turret.flightTime;
            double predictedY = target.y() + Math.sin(heading) * currentVelocity * robot.turret.flightTime;

            robot.turret.lock(new Pose(predictedX, predictedY, heading));
        });
    }

    public CommandBuilder autoLockSOTMAccel(Pose target, double maxVelocity) {
        return lockWhile(() -> distanceTraveledOnSegment() < 10, () -> {
            double heading = target.heading();
            double distTraveled = distanceTraveledOnSegment();

            double currentVelocity = maxVelocity * Math.min(1.0, distTraveled / 10.0);

            double predictedX = target.x() + Math.cos(heading) * currentVelocity * robot.turret.flightTime;
            double predictedY = target.y() + Math.sin(heading) * currentVelocity * robot.turret.flightTime;

            robot.turret.lock(new Pose(predictedX, predictedY, heading));
        });
    }

    private CommandBuilder lockWhile(BooleanSupplier condition, Runnable body) {
        return Command.build()
                .setExecute(() -> {
                    if (condition.getAsBoolean()) body.run();
                })
                .setDone(() -> !condition.getAsBoolean())
                .requiring(robot.turret);
    }

    // Pedro 3 leaves remainingDistance() stale after a path ends and currentCurve() NPEs on the last tick.
    private double distanceRemainingOnSegment() {
        return robot.follower.currentSegment() == null ? 0 : robot.follower.remainingDistance();
    }

    private double distanceTraveledOnSegment() {
        PathSegment segment = robot.follower.currentSegment();
        return segment == null ? 0 : segment.curve.length() - robot.follower.remainingDistance();
    }
}
