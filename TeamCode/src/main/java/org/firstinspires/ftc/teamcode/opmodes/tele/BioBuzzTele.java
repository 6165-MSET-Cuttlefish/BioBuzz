package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldPose;
import org.firstinspires.ftc.teamcode.architecture.input.EdgeBooleanSupplier;
import org.firstinspires.ftc.teamcode.architecture.input.GamepadPresses;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzOpMode;
import org.firstinspires.ftc.teamcode.biobuzz.PositionLock;
import org.firstinspires.ftc.teamcode.biobuzz.ShotTuning;
import org.firstinspires.ftc.teamcode.biobuzz.WaypointDrive;
import org.firstinspires.ftc.teamcode.biobuzz.modules.CellTipCamera;
import org.firstinspires.ftc.teamcode.biobuzz.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.pedro.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.pedro.PedroNeverDrives;

@Config("BioBuzz Tele")
@PedroNeverDrives
@TeleOp(name = "BioBuzz Tele", group = "A")
public class BioBuzzTele extends BioBuzzOpMode {

    // RED's coordinates; Tele can't take the Auto's pose yet.
    public static double startX = 30;
    public static double startY = 30;
    public static double startHeadingDeg = 90;
    public static WaypointDrive.Settings waypoint = new WaypointDrive.Settings();
    public static double takeoverStick = 0.2;
    public static int saveRumbleMs = 300;
    public static boolean cellTip = true;
    public static double rpmStep = 25;
    // About 1° of hood: one unit of hood position is about 31.5° (docs/shot-calculation-findings.md).
    public static double hoodStep = 0.03;
    public static double stickPress = 0.5;
    public static int layerRumbleMs = 150;
    public static long rumbleCooldownMs = 250;

    private static final double SLOW_MULTIPLIER = 0.75;

    private ActionLayer d1Layer = ActionLayer.TELE;
    private ActionLayer d2Layer = ActionLayer.MIXED;
    private double slowMultiplier = 1.0;
    private long lastGamepad1RumbleNs;
    private long lastGamepad2RumbleNs;

    private Pose shotWaypoint;
    private WaypointDrive waypointDrive;
    private boolean driverCancelled;
    private String waypointNote = "A saves one";

    private PositionLock positionLock;
    private String lockNote = "RB locks";

    private EdgeBooleanSupplier d2RsUp, d2RsDown, d2RsRight, d2RsLeft;
    private EdgeBooleanSupplier d2LsUp, d2LsDown, d2LsRight, d2LsLeft;

    @Override
    protected void initialize() {
        robot.follower.setPose(FieldPose.forAlliance(startX, startY, Math.toRadians(startHeadingDeg)));
        disableBallVision();
        applyCellTipFlag();
        setupStickPresses();
    }

    @Override
    protected void initializeLoop() {
        invalidateStickPresses();
    }

    @Override
    protected void onLoopStart() {
        applyCellTipFlag();
    }

    private static void applyCellTipFlag() {
        (cellTip ? CellTipCamera.VisionState.ENABLED : CellTipCamera.VisionState.DISABLED).activate();
    }

    @Override
    protected void onStart() {
        robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GamepadPresses.clear(gamepad1);
        GamepadPresses.clear(gamepad2);
    }

    @Override
    protected void gameLoop() {
        updateInputs();
        if (d1Layer == ActionLayer.TELE) drive();
    }

    private void setBothLayers(ActionLayer layer) {
        setD1Layer(layer);
        setD2Layer(layer);
    }

    private void setD1Layer(ActionLayer layer) {
        if (layer == d1Layer) return;
        d1Layer = layer;
        GamepadPresses.clear(gamepad1);
        rumbleGamepad1(layerRumbleMs);
    }

    private void setD2Layer(ActionLayer layer) {
        if (layer == d2Layer) return;
        d2Layer = layer;
        GamepadPresses.clear(gamepad2);
        rumbleGamepad2(layerRumbleMs);
    }

    private void updateInputs() {
        invalidateStickPresses();

        switch (d1Layer) {
            case TELE:
                d1TeleControls();
                break;
            case WAYPOINT_DRIVE:
                d1WaypointDriveControls();
                break;
            case POSITION_LOCK:
                d1PositionLockControls();
                break;
            default:
                throw new IllegalStateException("D1 has no controls for layer " + d1Layer);
        }
        switch (d2Layer) {
            case POLLEN_ONLY:
                d2PollenOnlyControls();
                break;
            case MIXED:
                d2MixedControls();
                break;
            default:
                throw new IllegalStateException("D2 has no controls for layer " + d2Layer);
        }
    }

    private void d1TeleControls() {
        if (gamepad1.leftStickButtonWasPressed()) {
            slowMultiplier = slowMultiplier == SLOW_MULTIPLIER ? 1.0 : SLOW_MULTIPLIER;
        }
        if (gamepad1.leftBumperWasPressed()) {
            if (robot.drivetrain.isHeadingLocked()) robot.drivetrain.unlockHeading();
            else robot.drivetrain.lockHeading();
        }
        if (gamepad1.aWasPressed()) saveShotWaypoint();
        // Last: starting a lock or a drive switches layers, which clears the presses still unread.
        if (gamepad1.rightBumperWasPressed()) startPositionLock();
        else if (gamepad1.leftTriggerWasPressed()) startWaypointDrive();
    }

    private void d1WaypointDriveControls() {
        if (Scheduler.isScheduled(waypointDrive)) {
            if (gamepad1.leftTriggerWasPressed()) cancelWaypointDrive("cancelled with LT");
            else if (sticksMoved()) cancelWaypointDrive("a stick moved");
        }
        if (!Scheduler.isScheduled(waypointDrive)) endWaypointDrive();
    }

    private void d1PositionLockControls() {
        if (Scheduler.isScheduled(positionLock)) {
            if (gamepad1.rightBumperWasPressed()) releasePositionLock("released with RB");
            else if (sticksMoved()) releasePositionLock("released: a stick moved");
        }
        if (!Scheduler.isScheduled(positionLock)) endPositionLock();
    }

    private void startPositionLock() {
        if (!BioBuzzConstants.isMeasured()) {
            lockNote = "waits for the robot's odometry to be measured (docs/odometry-tuning.md)";
            gamepad1.rumbleBlips(3);
            return;
        }
        positionLock = robot.actions.lockPosition(robot.follower.pose());
        Scheduler.schedule(positionLock);
        if (!Scheduler.isScheduled(positionLock)) {
            throw new IllegalStateException("the position lock wasn't scheduled: another command holds the follower or drivetrain");
        }
        setD1Layer(ActionLayer.POSITION_LOCK);
    }

    private void releasePositionLock(String note) {
        lockNote = note;
        positionLock.release();
    }

    private void endPositionLock() {
        // Pushed off and released, the robot may not face the heading lock's old target.
        if (robot.drivetrain.isHeadingLocked()) robot.drivetrain.lockHeading();
        setD1Layer(ActionLayer.TELE);
        if (positionLock.stopReason() != null) {
            lockNote = "stopped: " + positionLock.stopReason();
            gamepad1.rumbleBlips(3);
        }
    }

    private void saveShotWaypoint() {
        Pose pose = robot.follower.pose();
        String problem = WaypointDrive.poseProblem(pose, waypoint);
        if (problem != null) {
            waypointNote = "not saved: " + problem;
            gamepad1.rumbleBlips(3);
            return;
        }
        shotWaypoint = pose;
        waypointNote = "saved";
        rumbleGamepad1(saveRumbleMs);
    }

    private void startWaypointDrive() {
        if (!BioBuzzConstants.isMeasured()) {
            waypointNote = "waits for the robot's odometry to be measured (docs/odometry-tuning.md)";
            gamepad1.rumbleBlips(3);
            return;
        }
        if (shotWaypoint == null) {
            waypointNote = "none saved: A saves one";
            gamepad1.rumbleBlips(3);
            return;
        }
        waypointDrive = robot.actions.driveTo(shotWaypoint, waypoint);
        driverCancelled = false;
        Scheduler.schedule(waypointDrive);
        if (!Scheduler.isScheduled(waypointDrive)) {
            throw new IllegalStateException("the waypoint drive wasn't scheduled: another command holds the follower or drivetrain");
        }
        setD1Layer(ActionLayer.WAYPOINT_DRIVE);
    }

    private void cancelWaypointDrive(String reason) {
        driverCancelled = true;
        waypointDrive.cancel(reason);
    }

    private void endWaypointDrive() {
        // The lock's old heading would turn the robot off the waypoint's.
        if (robot.drivetrain.isHeadingLocked()) robot.drivetrain.lockHeading();
        setD1Layer(ActionLayer.TELE);
        switch (waypointDrive.status()) {
            case DONE:
                waypointNote = "arrived";
                break;
            case REFUSED:
                waypointNote = "refused: " + waypointDrive.detail();
                break;
            default:
                waypointNote = "stopped: " + waypointDrive.detail();
                break;
        }
        if (!driverCancelled && waypointDrive.status() != WaypointDrive.Status.DONE) gamepad1.rumbleBlips(3);
    }

    private boolean sticksMoved() {
        return Math.abs(gamepad1.left_stick_x) > takeoverStick || Math.abs(gamepad1.left_stick_y) > takeoverStick
                || Math.abs(gamepad1.right_stick_x) > takeoverStick || Math.abs(gamepad1.right_stick_y) > takeoverStick;
    }

    private void d2PollenOnlyControls() {
        if (gamepad2.rightBumperWasPressed()) {
            setD2Layer(ActionLayer.MIXED);
            return;
        }
        trimPollen();
    }

    private void d2MixedControls() {
        if (gamepad2.leftBumperWasPressed()) {
            setD2Layer(ActionLayer.POLLEN_ONLY);
            return;
        }
        trimPollen();
        trimNectar();
    }

    private void trimPollen() {
        trim(ShotTuning.pollenShooter, d2RsUp, d2RsDown, d2RsRight, d2RsLeft);
    }

    private void trimNectar() {
        trim(ShotTuning.nectarShooter, d2LsUp, d2LsDown, d2LsRight, d2LsLeft);
    }

    private static void trim(ShotTuning.ShooterTuning shooter, EdgeBooleanSupplier rpmUp, EdgeBooleanSupplier rpmDown,
                             EdgeBooleanSupplier hoodUp, EdgeBooleanSupplier hoodDown) {
        if (rpmUp.wasJustPressed()) shooter.rpmTrim += rpmStep;
        if (rpmDown.wasJustPressed()) shooter.rpmTrim -= rpmStep;
        if (hoodUp.wasJustPressed()) shooter.hoodTrim += hoodStep;
        if (hoodDown.wasJustPressed()) shooter.hoodTrim -= hoodStep;
    }

    private void setupStickPresses() {
        d2RsUp = new EdgeBooleanSupplier(() -> gamepad2.right_stick_y < -stickPress);
        d2RsDown = new EdgeBooleanSupplier(() -> gamepad2.right_stick_y > stickPress);
        d2RsRight = new EdgeBooleanSupplier(() -> gamepad2.right_stick_x > stickPress);
        d2RsLeft = new EdgeBooleanSupplier(() -> gamepad2.right_stick_x < -stickPress);
        d2LsUp = new EdgeBooleanSupplier(() -> gamepad2.left_stick_y < -stickPress);
        d2LsDown = new EdgeBooleanSupplier(() -> gamepad2.left_stick_y > stickPress);
        d2LsRight = new EdgeBooleanSupplier(() -> gamepad2.left_stick_x > stickPress);
        d2LsLeft = new EdgeBooleanSupplier(() -> gamepad2.left_stick_x < -stickPress);
    }

    // Every loop, read or not, so a stick held in a layer that ignores it doesn't fire once its layer reads it again.
    private void invalidateStickPresses() {
        d2RsUp.invalidate();
        d2RsDown.invalidate();
        d2RsRight.invalidate();
        d2RsLeft.invalidate();
        d2LsUp.invalidate();
        d2LsDown.invalidate();
        d2LsRight.invalidate();
        d2LsLeft.invalidate();
    }

    private void rumbleGamepad1(int durationMs) {
        long now = System.nanoTime();
        if (now - lastGamepad1RumbleNs >= rumbleCooldownMs * 1_000_000L) {
            gamepad1.rumble(durationMs);
            lastGamepad1RumbleNs = now;
        }
    }

    private void rumbleGamepad2(int durationMs) {
        long now = System.nanoTime();
        if (now - lastGamepad2RumbleNs >= rumbleCooldownMs * 1_000_000L) {
            gamepad2.rumble(durationMs);
            lastGamepad2RumbleNs = now;
        }
    }

    private void drive() {
        double forward = -gamepad1.left_stick_y * slowMultiplier;
        double strafe = gamepad1.left_stick_x * slowMultiplier;
        double turn = gamepad1.right_stick_x * slowMultiplier;
        robot.drivetrain.setMecanumTargets(forward, strafe, turn, false);
    }

    @Override
    protected void telemetry() {
        telemetry.addData("D1 Layer", d1Layer);
        telemetry.addData("D2 Layer", d2Layer);
        telemetry.addData("Slow Mode", slowMultiplier < 1.0 ? "75%" : "OFF");
        if (Drivetrain.headingLockAvailable()) {
            telemetry.addData("Heading Lock", robot.drivetrain.isHeadingLocked() ? "ON" : "OFF");
        } else {
            telemetry.addLine("The heading lock (LB) waits for the robot's odometry to be measured (docs/odometry-tuning.md)");
        }
        if (shotWaypoint == null) {
            telemetry.addData("Shot Waypoint", "none");
        } else {
            telemetry.addData("Shot Waypoint", "(%.1f, %.1f) facing %.0f deg",
                    shotWaypoint.x(), shotWaypoint.y(), Math.toDegrees(shotWaypoint.heading()));
        }
        if (d1Layer == ActionLayer.WAYPOINT_DRIVE) {
            telemetry.addData("Waypoint Drive", "%s %s", waypointDrive.status(), waypointDrive.phase());
        } else {
            telemetry.addData("Waypoint Drive", waypointNote);
        }
        if (d1Layer == ActionLayer.POSITION_LOCK) {
            Pose lock = positionLock.pose();
            Pose now = robot.follower.pose();
            telemetry.addData("Position Lock", "holding (%.1f, %.1f) facing %.0f deg; off by %.1f in, %.0f deg",
                    lock.x(), lock.y(), Math.toDegrees(lock.heading()), now.distance(lock),
                    Math.toDegrees(Angle.normalizeSigned(now.heading() - lock.heading())));
        } else {
            telemetry.addData("Position Lock", lockNote);
        }
        telemetry.addData("Pollen Trim", "%+.0f RPM, hood %+.3f",
                ShotTuning.pollenShooter.rpmTrim, ShotTuning.pollenShooter.hoodTrim);
        telemetry.addData("Nectar Trim", "%+.0f RPM, hood %+.3f",
                ShotTuning.nectarShooter.rpmTrim, ShotTuning.nectarShooter.hoodTrim);
    }
}
