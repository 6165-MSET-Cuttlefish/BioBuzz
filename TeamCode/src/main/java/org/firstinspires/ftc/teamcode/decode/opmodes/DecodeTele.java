package org.firstinspires.ftc.teamcode.decode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.input.EdgeBooleanSupplier;
import org.firstinspires.ftc.teamcode.architecture.input.LayerGamepad;
import org.firstinspires.ftc.teamcode.decode.DecodeOpMode;
import org.firstinspires.ftc.teamcode.decode.modules.Magazine;
import org.firstinspires.ftc.teamcode.decode.modules.Shooter;
import org.firstinspires.ftc.teamcode.decode.modules.ShooterInterpolation;
import org.firstinspires.ftc.teamcode.decode.modules.Turret;

import static org.firstinspires.ftc.teamcode.architecture.core.Context.allianceColor;
import static org.firstinspires.ftc.teamcode.architecture.telemetry.HtmlFormatter.COLOR_BLUE;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.blueTargetPose;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.redTargetPose;

@Config("Decode Tele")
@TeleOp(name = "Decode Tele", group = "Test")
public class DecodeTele extends DecodeOpMode {

    public static boolean optimizeRumbleCooldown = true;
    public static long optimizeRumbleCooldownMs = 250;

    private static final double SLOW_MULTIPLIER = 0.75;

    private boolean slowMode = false;

    private final long[] lastGamepad1RumbleMs = {0};
    private final long[] lastGamepad2RumbleMs = {0};
    private String pendingRelocalizeStatus = null;

    private LayerGamepad d1;
    private LayerGamepad d2;

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
        drive();
        getProfiler().leaveSection("tele.drive", stamp);
    }

    // Emitted here, not from gameLoop: dashboard data added before updateTelemetry() lands in the already-sent packet.
    @Override
    protected void telemetry() {
        long stamp = getProfiler().enterSection();
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

    private void updateInputs() {
        long stamp = getProfiler().enterSection();
        d1.invalidateAll();
        d2.invalidateAll();
        d1Lt.invalidate();
        d1Rt.invalidate();
        d2Lt.invalidate();
        d2Rt.invalidate();
        d2RsUp.invalidate();
        d2RsDown.invalidate();
        d2RsRight.invalidate();
        d2RsLeft.invalidate();
        getProfiler().leaveSection("tele.invalidateInputs", stamp);

        stamp = getProfiler().enterSection();
        d1Controls();
        getProfiler().leaveSection("tele.d1Controls", stamp);

        stamp = getProfiler().enterSection();
        d2Controls();
        getProfiler().leaveSection("tele.d2Controls", stamp);
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
        d1 = new LayerGamepad(gamepad1);
        d2 = new LayerGamepad(gamepad2);

        d1Lt = d1.leftTrigger.greaterThan(0.5);
        d1Rt = d1.rightTrigger.greaterThan(0.5);
        d2Lt = d2.leftTrigger.greaterThan(0.5);
        d2Rt = d2.rightTrigger.greaterThan(0.5);

        d2RsUp = d2.rightStickY.lessThan(-0.5);
        d2RsDown = d2.rightStickY.greaterThan(0.5);
        d2RsRight = d2.rightStickX.greaterThan(0.5);
        d2RsLeft = d2.rightStickX.lessThan(-0.5);
    }

    private void d1Controls() {
        if (d1Rt.wasJustPressed()
                && robot.magazine.getState(Magazine.HorizontalBackState.class) == Magazine.HorizontalBackState.OPEN) {
            Magazine.HorizontalBackState.OPEN_SHOOT.activate();
        }
        if (d1Rt.wasJustReleased()) {
            Magazine.VerticalState.HALF_DOWN.activate();
            Magazine.IntakeState.IDLE.activate();
            if (robot.magazine.getState(Magazine.HorizontalBackState.class) == Magazine.HorizontalBackState.OPEN_SHOOT) {
                Magazine.HorizontalBackState.OPEN.activate();
            }
        }
        if (d1Rt.getValue()) {
            Magazine.VerticalState.ON.activate();
            Magazine.IntakeState.SHOOTING.activate();
        } else if (d1.rightBumper.getValue()) {
            Magazine.VerticalState.HALF_DOWN.activate();
        }

        if (d1.leftBumper.wasJustPressed()) {
            if (robot.drivetrain.isHeadingLocked()) {
                robot.drivetrain.unlockHeading();
            } else {
                robot.drivetrain.lockHeading(allianceColor == AllianceColor.RED ? Math.toRadians(39.7) : Math.toRadians(155));
            }
        }

        if (d1Lt.wasJustPressed()) {
            slowMode = !slowMode;
        }

        boolean allHeld = d1.rightBumper.getValue() && d1.leftBumper.getValue()
                && d1.rightTrigger.getValue() > 0.1 && d1.leftTrigger.getValue() > 0.1;

        if (allHeld) {
            if (d1.b.getValue()) {
                Context.allianceColor = AllianceColor.RED;
            } else if (d1.x.getValue()) {
                Context.allianceColor = AllianceColor.BLUE;
            }
        } else {
            if (d1.x.wasJustPressed()) {
                resetPoseAndOffsets(new Pose(8.875, 9, Math.toRadians(180)));
            }
            if (d1.b.wasJustPressed()) {
                resetPoseAndOffsets(new Pose(141.5 - 8.875, 9, Math.toRadians(0)));
            }
            if (d1.y.wasJustPressed()) {
                resetPoseAndOffsets(new Pose(141.5 / 2, 8, Math.toRadians(90)));
            }
            if (d1.dpadDown.wasJustPressed()) {
                resetPoseAndOffsets(new Pose(141.5 / 2, 141.5 - 8.875, Math.toRadians(90)));
            }
        }
    }

    private void d2Controls() {
        if (d2.dpadUp.wasJustPressed()) {
            (robot.magazine.getState(Magazine.IntakeState.class) == Magazine.IntakeState.OFF
                    ? Magazine.IntakeState.IDLE
                    : Magazine.IntakeState.OFF).activate();
        }

        if (d2Rt.getValue()) {
            Magazine.IntakeState.FORWARD.activate();
        } else if (d2Lt.getValue()) {
            Magazine.IntakeState.REVERSE.activate();
        } else if (d2Rt.wasJustReleased() || d2Lt.wasJustReleased()) {
            Magazine.IntakeState.IDLE.activate();
        }

        if (d2.dpadDown.wasJustPressed()) {
            Magazine.VerticalState.OFF.activate();
        }

        if (d2.x.wasJustPressed()) {
            Shooter.FlywheelState.IDLE.activate();
            Shooter.HoodState.BOTTOM.activate();
        } else if (d2.y.wasJustPressed()) {
            Shooter.FlywheelState.OFF.activate();
            Shooter.HoodState.BOTTOM.activate();
        } else if (d2.a.wasJustPressed()) {
            ShooterInterpolation.activeMode = ShooterInterpolation.Mode.CLOSE;
            Shooter.FlywheelState.PID.activate();
            Shooter.HoodState.PID.activate();
            Turret.isCloseTele = true;
        } else if (d2.b.wasJustPressed()) {
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

        if (d2.leftStickX.getValue() > 0.5) {
            robot.turret.turretManualOffset -= .25;
            if (Turret.isCloseTele) {
                Turret.teleTurretOffsetClose = robot.turret.turretManualOffset;
            } else {
                Turret.teleTurretOffsetFar = robot.turret.turretManualOffset;
            }
        }
        if (d2.leftStickX.getValue() < -0.5) {
            robot.turret.turretManualOffset += .25;
            if (Turret.isCloseTele) {
                Turret.teleTurretOffsetClose = robot.turret.turretManualOffset;
            } else {
                Turret.teleTurretOffsetFar = robot.turret.turretManualOffset;
            }
        }

        if (d2.dpadRight.wasJustPressed()) {
            (robot.turret.getState(Turret.TurretState.class).equals(Turret.TurretState.AUTOAIM)
                    ? Turret.TurretState.HOLD
                    : Turret.TurretState.AUTOAIM).activate();
        }

        if (d2.dpadLeft.wasJustPressed()) {
            relocalizeFromLimelight();
        }
    }

    private void resetPoseAndOffsets(Pose pose) {
        robot.follower.setPose(pose);
        robot.shooter.closeVelocityOffset = 0;
        robot.shooter.farVelocityOffset = 0;
        robot.shooter.closeHoodOffset = 0;
        robot.shooter.farHoodOffset = 0;

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
            rumble(gamepad1, lastGamepad1RumbleMs, 200);
            pendingRelocalizeStatus = "SUCCESS";
        } else {
            pendingRelocalizeStatus = String.valueOf(robot.turret.getRelocalizationStatus());
        }
    }

    private void drive() {
        double forward = -d1.leftStickY.getValue();
        double strafe = d1.leftStickX.getValue();
        double turn = d1.rightStickX.getValue();

        if (slowMode) {
            forward *= SLOW_MULTIPLIER;
            strafe *= SLOW_MULTIPLIER;
            turn *= SLOW_MULTIPLIER;
        }

        robot.drivetrain.setMecanumTargets(forward, strafe, turn, false);
    }
}
