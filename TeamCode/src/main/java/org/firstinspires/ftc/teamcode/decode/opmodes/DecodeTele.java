package org.firstinspires.ftc.teamcode.decode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.input.EdgeBooleanSupplier;
import org.firstinspires.ftc.teamcode.architecture.input.LayerGamepad;
import org.firstinspires.ftc.teamcode.architecture.input.LayerStack;
import org.firstinspires.ftc.teamcode.architecture.input.LayeredGamepad;
import org.firstinspires.ftc.teamcode.architecture.prism.Color;
import org.firstinspires.ftc.teamcode.decode.DecodeContext;
import org.firstinspires.ftc.teamcode.decode.DecodeOpMode;
import org.firstinspires.ftc.teamcode.decode.modules.Endgame;
import org.firstinspires.ftc.teamcode.decode.modules.Magazine;
import org.firstinspires.ftc.teamcode.decode.modules.MagazineState;
import org.firstinspires.ftc.teamcode.decode.modules.Shooter;
import org.firstinspires.ftc.teamcode.decode.modules.ShooterInterpolation;
import org.firstinspires.ftc.teamcode.decode.modules.Turret;

import java.util.HashMap;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.architecture.core.Context.allianceColor;
import static org.firstinspires.ftc.teamcode.architecture.telemetry.HtmlFormatter.COLOR_BLUE;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.blueTargetPose;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.redTargetPose;
import static org.firstinspires.ftc.teamcode.decode.modules.MagazineState.ArtifactColor.GREEN;
import static org.firstinspires.ftc.teamcode.decode.modules.MagazineState.ArtifactColor.PURPLE;

@Config("Decode Tele")
@TeleOp(name = "Decode Tele", group = "Test")
public class DecodeTele extends DecodeOpMode {

    enum ActionLayer { TELE, SORT, ENDGAME }

    public static boolean soloDriver = false;
    public static boolean outOfRangeRedAndPreventShooting = false;

    public static boolean optimizeInputInvalidation = true;
    public static boolean optimizeRumbleCooldown = true;
    public static long optimizeRumbleCooldownMs = 250;

    private static final long OUT_OF_RANGE_SHOOT_DELAY_MS = 1000;
    private static final double SLOW_MULTIPLIER = 0.75;

    private ActionLayer d1Layer = ActionLayer.TELE;
    private ActionLayer d2Layer = ActionLayer.TELE;
    private boolean slowMode = false;

    private boolean lastIntakeBallDetected = false;
    private boolean lastPrismWarningActive = false;
    private long outOfRangeShootDelayStartMs = 0;
    private final long[] lastGamepad1RumbleMs = {0};
    private final long[] lastGamepad2RumbleMs = {0};
    private String pendingRelocalizeStatus = null;

    private LayeredGamepad<ActionLayer> d1;
    private LayeredGamepad<ActionLayer> d2;

    private EdgeBooleanSupplier d1Lt;
    private EdgeBooleanSupplier d1Rt;
    private EdgeBooleanSupplier d2Lt;
    private EdgeBooleanSupplier d2Rt;

    private EdgeBooleanSupplier d2RsUp;
    private EdgeBooleanSupplier d2RsDown;
    private EdgeBooleanSupplier d2RsRight;
    private EdgeBooleanSupplier d2RsLeft;

    @Override
    protected boolean shouldReadDuringInit() {
        return false;
    }

    @Override
    protected void initialize() {
        Turret.isAuto = false;
        setupGamepads();
        resetOffsets();
        Magazine.updateColorSensor = true;
        Magazine.updateDistanceSensor = true;
    }

    @Override
    protected void initializeLoop() {
        updateInputs();
    }

    @Override
    protected void onStart() {
        robot.drivetrain.getFl().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.drivetrain.getFr().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.drivetrain.getBl().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.drivetrain.getBr().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.targetPose = Context.allianceColor == AllianceColor.RED
                ? new Pose(redTargetPose.x() - 1, redTargetPose.y())
                : new Pose(blueTargetPose.x() + 1, blueTargetPose.y());
    }

    @Override
    protected void gameLoop() {
        long stamp = getProfiler().enterSection();
        updateInputs();
        getProfiler().leaveSection("tele.updateInputs", stamp);

        stamp = getProfiler().enterSection();
        if (!robot.endgame.getState(Endgame.FullLiftState.class).equals(Endgame.FullLiftState.FULL_LIFT)
                && !robot.endgame.getState(Endgame.LeftPtoState.class).equals(Endgame.LeftPtoState.DOWN)
                && !robot.endgame.getState(Endgame.RightPtoState.class).equals(Endgame.RightPtoState.DOWN)) {
            drive();
        }
        getProfiler().leaveSection("tele.drive", stamp);
    }

    // Emitted here, not from gameLoop: dashboard data added before updateTelemetry() lands in the already-sent packet.
    @Override
    protected void telemetry() {
        long stamp = getProfiler().enterSection();
        addDSLarge("D1 Layer", d1Layer);
        addDSLarge("D2 Layer", d2Layer);
        robot.telemetry.addSeparator();
        robot.telemetry.addGroupHeader("CONTROLS", COLOR_BLUE);
        robot.telemetry.addData("Slow Mode", slowMode ? "75%" : "OFF");
        robot.telemetry.addData("Heading Lock", robot.drivetrain.isHeadingLocked() ? "ON" : "OFF");
        if (pendingRelocalizeStatus != null) {
            addDSLarge("Relocalize", pendingRelocalizeStatus);
            pendingRelocalizeStatus = null;
        }
        getProfiler().leaveSection("tele.controlsTelemetry", stamp);
    }

    private void setBothLayers(ActionLayer layer) {
        d1Layer = layer;
        d2Layer = layer;
        d1.setLayer(layer);
        d2.setLayer(layer);
        invalidateAllKeyReaders();
    }

    private void setD1Layer(ActionLayer layer) {
        d1Layer = layer;
        d1.setLayer(layer);
        d1.invalidateAll();
    }

    private void setD2Layer(ActionLayer layer) {
        d2Layer = layer;
        d2.setLayer(layer);
        d2.invalidateAll();
        invalidateD2KeyReaders();
    }

    private void invalidateAllKeyReaders() {
        d1.invalidateAll();
        d2.invalidateAll();

        d1Lt.invalidate();
        d1Rt.invalidate();
        d2Lt.invalidate();
        d2Rt.invalidate();

        invalidateD2KeyReaders();
    }

    private void invalidateD2KeyReaders() {
        d2RsUp.invalidate();
        d2RsDown.invalidate();
        d2RsRight.invalidate();
        d2RsLeft.invalidate();
    }

    private void updateInputs() {
        long stamp = getProfiler().enterSection();
        if (!optimizeInputInvalidation) {
            invalidateAllKeyReaders();
        } else {
            invalidateActiveLayerKeyReaders();
        }
        getProfiler().leaveSection("tele.invalidateInputs", stamp);

        stamp = getProfiler().enterSection();
        if (d1Layer == ActionLayer.TELE) {
            d1TeleControls();
        }
        getProfiler().leaveSection("tele.d1Controls", stamp);

        stamp = getProfiler().enterSection();
        switch (d2Layer) {
            case TELE:
                d2TeleControls();
                break;
            case SORT:
                sortLayer();
                break;
            case ENDGAME:
                endgameLayer();
                break;
        }
        getProfiler().leaveSection("tele.d2Controls", stamp);
    }

    private void invalidateActiveLayerKeyReaders() {
        d1.invalidateActive();
        d2.invalidateActive();

        d1Lt.invalidate();
        d1Rt.invalidate();
        d2Lt.invalidate();
        d2Rt.invalidate();

        if (d2Layer == ActionLayer.TELE) {
            invalidateD2KeyReaders();
        }
    }

    private void rumble(Gamepad gamepad, long[] lastRumbleMs, int durationMs) {
        if (!optimizeRumbleCooldown) {
            gamepad.rumble(durationMs);
            return;
        }

        long now = System.currentTimeMillis();
        long cooldown = Math.max(0, optimizeRumbleCooldownMs);
        if (now - lastRumbleMs[0] >= cooldown) {
            gamepad.rumble(durationMs);
            lastRumbleMs[0] = now;
        }
    }

    private void setupGamepads() {
        Map<ActionLayer, LayerGamepad> d1Layers = new HashMap<>();
        d1Layers.put(ActionLayer.TELE, new LayerGamepad(gamepad1));
        d1Layers.put(ActionLayer.ENDGAME, new LayerGamepad(gamepad1));

        Map<ActionLayer, LayerGamepad> d2Layers = new HashMap<>();
        d2Layers.put(ActionLayer.TELE, new LayerGamepad(gamepad2));
        d2Layers.put(ActionLayer.SORT, new LayerGamepad(gamepad2));
        d2Layers.put(ActionLayer.ENDGAME, new LayerGamepad(gamepad2));

        d1 = new LayeredGamepad<>(new LayerStack<>(ActionLayer.TELE, d1Layers));
        d2 = new LayeredGamepad<>(new LayerStack<>(ActionLayer.TELE, d2Layers));

        d1.setLayer(ActionLayer.TELE);
        d2.setLayer(ActionLayer.TELE);

        d1Lt = d1.LT().greaterThan(0.5);
        d1Rt = d1.RT().greaterThan(0.5);
        d2Lt = d2.LT().greaterThan(0.5);
        d2Rt = d2.RT().greaterThan(0.5);

        d2RsUp = d2.getRightStickY().lessThan(-0.5);
        d2RsDown = d2.getRightStickY().greaterThan(0.5);
        d2RsRight = d2.getRightStickX().greaterThan(0.5);
        d2RsLeft = d2.getRightStickX().lessThan(-0.5);

        // Untracked derived suppliers aren't primed on a layer switch and fire a spurious edge.
        d1.track(d1Lt, d1Rt);
        d2.track(d2Lt, d2Rt, d2RsUp, d2RsDown, d2RsRight, d2RsLeft);
    }

    private void d1TeleControls() {
        boolean warningDetected = !robot.turret.isWithinRange() || !robot.shooter.isAtTargetVelocity();
        boolean warningActive = outOfRangeRedAndPreventShooting && warningDetected;
        long now = System.currentTimeMillis();

        if (d1.RB().getValue()) {
            Magazine.VerticalState.HALF_DOWN.activate();
        } else if (d1Rt.wasJustPressed()) {
            Magazine.HorizontalBackState backState = robot.magazine.getState(Magazine.HorizontalBackState.class);
            if (backState == Magazine.HorizontalBackState.OPEN) {
                Magazine.HorizontalBackState.OPEN_SHOOT.activate();
            } else if (backState == Magazine.HorizontalBackState.STORED) {
                Magazine.HorizontalBackState.STORED_SHOOT.activate();
            }

            if (warningActive) {
                outOfRangeShootDelayStartMs = now;
            }
        } else if (d1Rt.wasJustReleased()) {
            Magazine.VerticalState.HALF_DOWN.activate();
            Magazine.IntakeState.IDLE.activate();
            outOfRangeShootDelayStartMs = 0;

            if (robot.magazine.getState(Magazine.HorizontalBackState.class).equals(Magazine.HorizontalBackState.OPEN_SHOOT)) {
                Magazine.HorizontalBackState.OPEN.activate();
            } else if (robot.magazine.getState(Magazine.HorizontalBackState.class).equals(Magazine.HorizontalBackState.STORED_SHOOT)) {
                Magazine.HorizontalBackState.STORED.activate();
            }
        }

        if (d1Rt.getValue()) {
            if (warningActive && outOfRangeShootDelayStartMs == 0) {
                outOfRangeShootDelayStartMs = now;
            }

            boolean shouldDelayShot = warningActive
                    && (now - outOfRangeShootDelayStartMs) < OUT_OF_RANGE_SHOOT_DELAY_MS;

            if (shouldDelayShot) {
                Magazine.VerticalState.HALF_DOWN.activate();
                Magazine.IntakeState.IDLE.activate();
            } else {
                Magazine.VerticalState.ON.activate();
                Magazine.IntakeState.SHOOTING.activate();
            }
        }

        robot.magazine.setPrismFlash(d1Rt.getValue());

        if (d1.DPAD_RIGHT().wasJustPressed()) {
            Scheduler.schedule(robot.actions.shootAll(true, false));
        }

        if (d1.LB().wasJustPressed()) {
            if (soloDriver) {
                Magazine.IntakeState.FORWARD.activate();
            } else {
                if (robot.drivetrain.isHeadingLocked()) {
                    robot.drivetrain.unlockHeading();
                } else {
                    robot.drivetrain.lockHeading(allianceColor == AllianceColor.RED ? Math.toRadians(39.7) : Math.toRadians(155));
                }
            }
        }
        if (d1.LB().wasJustReleased() && soloDriver) {
            Magazine.IntakeState.IDLE.activate();
        }

        if (d1Lt.wasJustPressed()) {
            slowMode = !slowMode;
        }

        boolean allHeld = d1.RB().getValue() && d1.LB().getValue()
                && d1.RT().getValue() > 0.1 && d1.LT().getValue() > 0.1;

        if (allHeld) {
            if (d1.B().getValue()) {
                Context.allianceColor = AllianceColor.RED;
            } else if (d1.X().getValue()) {
                Context.allianceColor = AllianceColor.BLUE;
            }
        } else {
            if (d1.X().wasJustPressed()) {
                resetPoseAndOffsets(new Pose(8.875, 9, Math.toRadians(180)));
            }
            if (d1.B().wasJustPressed()) {
                resetPoseAndOffsets(new Pose(141.5 - 8.875, 9, Math.toRadians(0)));
            }
            if (d1.Y().wasJustPressed()) {
                resetPoseAndOffsets(new Pose(141.5 / 2, 8, Math.toRadians(90)));
            }
            if (d1.DPAD_DOWN().wasJustPressed()) {
                resetPoseAndOffsets(new Pose(141.5 / 2, 141.5 - 8.875, Math.toRadians(90)));
            }
        }

        if (warningActive && !lastPrismWarningActive) {
            rumble(gamepad1, lastGamepad1RumbleMs, 200);
        }
        lastPrismWarningActive = warningActive;

        if (!d1Rt.getValue()) {
            if (warningActive) {
                robot.magazine.setStatusPrismColor(Color.RED);
            } else if (slowMode) {
                robot.magazine.setStatusPrismColor(Color.WHITE);
            } else if (robot.drivetrain.isHeadingLocked()) {
                robot.magazine.setStatusPrismColor(Color.PURPLE);
            } else {
                robot.magazine.setStatusPrismSnake();
            }
        }

        if (Magazine.updateColorSensor) {
            MagazineState colorState = robot.magazine.getMagazineState();
            if (robot.magazine.intakeIsFull) {
                Magazine.HeadlightFrontState.CYAN.activate();
                Magazine.HeadlightMiddleState.CYAN.activate();
                Magazine.HeadlightBackState.CYAN.activate();
            } else {
                (colorState.getPosition3() == GREEN
                        ? Magazine.HeadlightFrontState.GREEN
                        : colorState.getPosition3() == PURPLE
                        ? Magazine.HeadlightFrontState.PURPLE
                        : Magazine.HeadlightFrontState.OFF).activate();
                (colorState.getPosition2() == GREEN
                        ? Magazine.HeadlightMiddleState.GREEN
                        : colorState.getPosition2() == PURPLE
                        ? Magazine.HeadlightMiddleState.PURPLE
                        : robot.magazine.isMiddleSlotFilled()
                        ? Magazine.HeadlightMiddleState.WHITE
                        : Magazine.HeadlightMiddleState.OFF).activate();
                (colorState.getPosition1() == GREEN
                        ? Magazine.HeadlightBackState.GREEN
                        : colorState.getPosition1() == PURPLE
                        ? Magazine.HeadlightBackState.PURPLE
                        : robot.magazine.isBackSlotFilled()
                        ? Magazine.HeadlightBackState.WHITE
                        : Magazine.HeadlightBackState.OFF).activate();
            }
        } else {
            if (robot.magazine.intakeIsFull) {
                Magazine.HeadlightFrontState.CYAN.activate();
                Magazine.HeadlightMiddleState.CYAN.activate();
                Magazine.HeadlightBackState.CYAN.activate();
            } else {
                Magazine.HeadlightFrontState.OFF.activate();
                (robot.magazine.isMiddleSlotFilled()
                        ? Magazine.HeadlightMiddleState.PURPLE
                        : Magazine.HeadlightMiddleState.OFF).activate();
                (robot.magazine.isBackSlotFilled()
                        ? Magazine.HeadlightBackState.PURPLE
                        : Magazine.HeadlightBackState.OFF).activate();
            }
        }

        if (robot.magazine.intakeIsFull != lastIntakeBallDetected) {
            if (robot.magazine.intakeIsFull) {
                rumble(gamepad1, lastGamepad1RumbleMs, 500);
            }
        }
        lastIntakeBallDetected = robot.magazine.intakeIsFull;
    }

    private void d2TeleControls() {
        if (d2.LSB().wasJustPressed()) {
            Scheduler.schedule(robot.actions.wiggleFrontHzPusher());
        }
        if (d2.RSB().wasJustPressed()) {
            Scheduler.schedule(robot.actions.wiggleBackHzPusher());
        }

        if (d2.DPAD_UP().wasJustPressed()) {
            if (!robot.magazine.getState(Magazine.IntakeState.class).equals(Magazine.IntakeState.OFF)) {
                Magazine.IntakeState.OFF.activate();
            } else {
                Magazine.IntakeState.IDLE.activate();
            }
        }

        if (d2.X().wasJustPressed()) {
            Shooter.FlywheelState.IDLE.activate();
            Shooter.HoodState.BOTTOM.activate();
        } else if (d2.Y().wasJustPressed()) {
            Shooter.FlywheelState.OFF.activate();
            Shooter.HoodState.BOTTOM.activate();
        } else if (d2.A().wasJustPressed()) {
            ShooterInterpolation.activeMode = ShooterInterpolation.Mode.CLOSE;
            Shooter.FlywheelState.PID.activate();
            Shooter.HoodState.PID.activate();
            Turret.isCloseTele = true;
        } else if (d2.B().wasJustPressed()) {
            ShooterInterpolation.activeMode = ShooterInterpolation.Mode.FAR;
            Shooter.FlywheelState.PID.activate();
            Shooter.HoodState.PID.activate();
            Turret.isCloseTele = false;
        }

        if (d2RsUp.wasJustPressed()) {
            if (ShooterInterpolation.activeMode == ShooterInterpolation.Mode.CLOSE) {
                robot.shooter.closeVelocityOffset += 25;
            } else {
                robot.shooter.farVelocityOffset += 25;
            }
        } else if (d2RsDown.wasJustPressed()) {
            if (ShooterInterpolation.activeMode == ShooterInterpolation.Mode.CLOSE) {
                robot.shooter.closeVelocityOffset -= 25;
            } else {
                robot.shooter.farVelocityOffset -= 25;
            }
        }
        if (d2RsRight.wasJustPressed()) {
            if (ShooterInterpolation.activeMode == ShooterInterpolation.Mode.CLOSE) {
                robot.shooter.closeHoodOffset += 0.01;
            } else {
                robot.shooter.farHoodOffset += 0.01;
            }
        } else if (d2RsLeft.wasJustPressed()) {
            if (ShooterInterpolation.activeMode == ShooterInterpolation.Mode.CLOSE) {
                robot.shooter.closeHoodOffset -= 0.01;
            } else {
                robot.shooter.farHoodOffset -= 0.01;
            }
        }

        if (robot.turret.getState(Turret.TurretState.class) == Turret.TurretState.HOLD) {
            rumble(gamepad2, lastGamepad2RumbleMs, 200);
        }

        if (d2Rt.getValue()) {
            Magazine.IntakeState.FORWARD.activate();
        }

        if (d2Rt.wasJustReleased()) {
            Magazine.IntakeState.IDLE.activate();
        }

        if (d2.getLeftStickX().getValue() > 0.5) {
            robot.turret.turretManualOffset -= .25;
            if (Turret.isCloseTele) {
                Turret.teleTurretOffsetClose = robot.turret.turretManualOffset;
            } else {
                Turret.teleTurretOffsetFar = robot.turret.turretManualOffset;
            }
        }
        if (d2.getLeftStickX().getValue() < -0.5) {
            robot.turret.turretManualOffset += .25;
            if (Turret.isCloseTele) {
                Turret.teleTurretOffsetClose = robot.turret.turretManualOffset;
            } else {
                Turret.teleTurretOffsetFar = robot.turret.turretManualOffset;
            }
        }

        if (d2Lt.wasJustPressed()) {
            Magazine.IntakeState.ANALOG_EXTAKE.setValue(-d2.LT().getValue());
            Magazine.IntakeState.ANALOG_EXTAKE.activate();
        }
        if (d2Lt.wasJustReleased()) {
            Magazine.IntakeState.IDLE.activate();
        }

        if (d2.DPAD_DOWN().wasJustPressed()) {
            Magazine.VerticalState.OFF.activate();
        }

        if (d2.DPAD_RIGHT().wasJustPressed()) {
            (robot.turret.getState(Turret.TurretState.class).equals(Turret.TurretState.AUTOAIM)
                    ? Turret.TurretState.HOLD
                    : Turret.TurretState.AUTOAIM).activate();
        }

        if (d2.DPAD_LEFT().wasJustPressed()) {
            relocalizeFromLimelight();
        }

        if (d2.LB().wasJustPressed()) {
            setD1Layer(ActionLayer.TELE);
            setD2Layer(ActionLayer.SORT);
            rumble(gamepad2, lastGamepad2RumbleMs, 150);
            return;
        }
        if (d2.RB().wasJustPressed()) {
            setBothLayers(ActionLayer.ENDGAME);
            rumble(gamepad2, lastGamepad2RumbleMs, 150);
        }
    }

    private void sortLayer() {
        rumble(gamepad2, lastGamepad2RumbleMs, 200);

        if (d2.LB().wasJustPressed()) {
            setBothLayers(ActionLayer.TELE);
            rumble(gamepad2, lastGamepad2RumbleMs, 150);
            return;
        }
        if (d2.RB().wasJustPressed()) {
            setBothLayers(ActionLayer.ENDGAME);
            rumble(gamepad2, lastGamepad2RumbleMs, 150);
            return;
        }

        if (d2.LSB().wasJustPressed()) {
            (robot.magazine.getState(Magazine.HorizontalBackState.class)
                    .equals(Magazine.HorizontalBackState.OPEN)
                    ? Magazine.HorizontalBackState.STORED
                    : Magazine.HorizontalBackState.OPEN).activate();
        }

        if (d2.RSB().wasJustPressed()) {
            (robot.magazine.getState(Magazine.HorizontalFrontState.class)
                    .equals(Magazine.HorizontalFrontState.OPEN)
                    ? Magazine.HorizontalFrontState.STORED
                    : Magazine.HorizontalFrontState.OPEN).activate();
        }

        if (d2Rt.getValue()) {
            Magazine.IntakeState.FORWARD.activate();
        }

        if (d2Rt.wasJustReleased()) {
            Magazine.IntakeState.IDLE.activate();
        }

        if (d2Lt.getValue()) {
            Magazine.IntakeState.REVERSE.activate();
        }

        if (d2Lt.wasJustReleased()) {
            Magazine.IntakeState.IDLE.activate();
        }

        if (d2.X().wasJustPressed()) {
            Scheduler.schedule(robot.actions.sortMagazine());
        }

        if (d2.Y().wasJustPressed()) {
            Magazine.HorizontalFrontState.OPEN.activate();
            Magazine.HorizontalBackState.OPEN.activate();
        }

        if (d2.DPAD_UP().getValue()) {
            DecodeContext.motif = new MagazineState(PURPLE, PURPLE, GREEN);
        }

        if (d2.DPAD_DOWN().getValue()) {
            DecodeContext.motif = new MagazineState(PURPLE, GREEN, PURPLE);
        }

        if (d2.DPAD_LEFT().getValue()) {
            DecodeContext.motif = new MagazineState(GREEN, PURPLE, PURPLE);
        }
    }

    private void resetPoseAndOffsets(Pose pose) {
        robot.follower.setPose(pose);
        robot.shooter.closeVelocityOffset = 0;
        robot.shooter.farVelocityOffset = 0;
        robot.shooter.closeHoodOffset = 0;
        robot.shooter.farHoodOffset = 0;
        robot.turret.turretAprilTagOffset = 0;

        resetOffsets();
    }

    private void resetOffsets() {
        if (Context.allianceColor == AllianceColor.RED) {
            Turret.teleTurretOffsetClose = -4.8;
            Turret.teleTurretOffsetFar = -4.4;
        } else {
            Turret.teleTurretOffsetClose = 1.8;
            Turret.teleTurretOffsetFar = 4.5;
        }
    }

    private void relocalizeFromLimelight() {
        Pose relocalizedPose = robot.turret.getRelocalizedRobotPoseFromLimelight();
        if (relocalizedPose != null) {
            // Pose only; the shooter/turret tuning offsets are deliberately kept.
            robot.follower.setPose(new Pose(relocalizedPose.x(), relocalizedPose.y(), robot.follower.pose().heading()));
            robot.turret.turretAprilTagOffset = 0;
            rumble(gamepad1, lastGamepad1RumbleMs, 200);
            pendingRelocalizeStatus = "SUCCESS";
        } else {
            pendingRelocalizeStatus = String.valueOf(robot.turret.getRelocalizationStatus());
        }
    }

    private void endgameLayer() {
        rumble(gamepad2, lastGamepad2RumbleMs, 200);

        if (d2.A().wasJustPressed()) {
            Endgame.InitialState.LIFT.activate();
        }
        if (d2.X().wasJustPressed()) {
            Scheduler.schedule(robot.actions.endgameSequence());
        }
        if (d2.LB().wasJustPressed()) {
            setBothLayers(ActionLayer.TELE);
            rumble(gamepad2, lastGamepad2RumbleMs, 150);
            return;
        }
        if (d2.Y().wasJustPressed()) {
            Scheduler.schedule(robot.actions.cancelEndgameSequence());
        }
        if (d2.DPAD_RIGHT().wasJustPressed()) {
            Magazine.VerticalState.OFF.activate();
        }
    }

    private void drive() {
        double forward = -d1.getLeftStickY().getValue();
        double strafe = d1.getLeftStickX().getValue();
        double turn = d1.getRightStickX().getValue();

        if (slowMode) {
            forward *= SLOW_MULTIPLIER;
            strafe *= SLOW_MULTIPLIER;
            turn *= SLOW_MULTIPLIER;
        }

        robot.drivetrain.setMecanumTargets(forward, strafe, turn, false);
    }
}
