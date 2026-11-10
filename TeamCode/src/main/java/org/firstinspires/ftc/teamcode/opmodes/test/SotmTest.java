package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldPose;
import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.input.EdgeBooleanSupplier;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzOpMode;
import org.firstinspires.ftc.teamcode.biobuzz.ShooterGeometry;
import org.firstinspires.ftc.teamcode.biobuzz.ShotSolver;
import org.firstinspires.ftc.teamcode.biobuzz.ShotTuning;
import org.firstinspires.ftc.teamcode.biobuzz.TargetSelector;
import org.firstinspires.ftc.teamcode.biobuzz.modules.CellTipCamera;
import org.firstinspires.ftc.teamcode.biobuzz.modules.Shooter;
import org.firstinspires.ftc.teamcode.biobuzz.modules.Turret;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

@TeleOp(name = "SOTM Test", group = "Test")
@Config("SOTM Test")
public class SotmTest extends BioBuzzOpMode {

    /** LEGACY: the turrets run Turret's own AUTOAIM and the shot is solved as if standing still. */
    public enum Aim { SOTM, LEGACY }

    /** RED coordinates; mapped for BLUE. */
    public static double startX = 30;
    public static double startY = 30;
    public static double startHeadingDeg = 90;
    public static Aim aim = Aim.SOTM;
    public static boolean pollenEnabled = true;
    public static boolean nectarEnabled = true;
    /** On: aim at the CELL facing the robot's half of the field. Off: X and the webcam choose. */
    public static boolean cellByFieldHalf = true;
    /** With cellByFieldHalf off, flip the target on each TIP the webcam sees. */
    public static boolean flipOnCameraTip = false;

    private static final int POLLEN = 0;
    private static final int NECTAR = 1;

    private Turret turret;
    private Shooter shooter;
    private ShotSolver solver;
    private final TargetSelector selector = new TargetSelector();
    private final ShotSolver.Motion motion = new ShotSolver.Motion();
    private final double[] estimatedTurretDeg = new double[2];
    private final boolean[] ready = new boolean[2];
    private ShotSolver.Result[] results;
    private double lastS = Double.NaN;
    private boolean flywheels;
    private boolean firing;
    private double lastAllReadyS = Double.NEGATIVE_INFINITY;

    private EdgeBooleanSupplier toggleCell;
    private EdgeBooleanSupplier toggleSotm;

    @Override
    protected void initialize() {
        disableBallVision();
        applyCellTipFlag();
        robot.follower.setPose(FieldPose.forAlliance(startX, startY, Math.toRadians(startHeadingDeg)));
        turret = new Turret(hardwareMap, robot.follower);
        shooter = new Shooter(hardwareMap);
        turret.setGoal(() -> {
            double[] p = BioBuzzField.cellOpening(selector.upCell()).aimPoint(ShotTuning.aimDepthIn);
            return new Pose(p[0], p[1]);
        });
        solver = new ShotSolver(ShooterGeometry.pollen(), ShooterGeometry.nectar());
        solver.setTuning(POLLEN, ShotTuning.pollenShooter);
        solver.setTuning(NECTAR, ShotTuning.nectarShooter);
        toggleCell = new EdgeBooleanSupplier(() -> gamepad1.x);
        toggleSotm = new EdgeBooleanSupplier(() -> gamepad1.y);
    }

    @Override
    protected void initializeLoop() {
        toggleCell.invalidate();
        toggleSotm.invalidate();
    }

    @Override
    protected void onLoopStart() {
        applyCellTipFlag();
    }

    private static void applyCellTipFlag() {
        (flipOnCameraTip && !cellByFieldHalf ? CellTipCamera.VisionState.ENABLED : CellTipCamera.VisionState.DISABLED).activate();
    }

    @Override
    protected void onStart() {
        robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setState(aim == Aim.SOTM ? Turret.TurretState.SOTM : Turret.TurretState.AUTOAIM);
    }

    @Override
    protected void gameLoop() {
        boolean blue = Context.allianceColor == AllianceColor.BLUE;
        if (cellByFieldHalf) {
            selector.followHalf(robot.follower.pose().y(), FieldConfig.fieldWidthInches, blue, ShotTuning.cellSwitchBandIn);
        } else {
            if (toggleCell.wasJustPressed()) selector.toggle();
            if (flipOnCameraTip) selector.onTipVerdict(robot.cellTip.isTipped());
        }
        if (toggleSotm.wasJustPressed()) ShotTuning.sotmEnabled = !ShotTuning.sotmEnabled;
        if (gamepad1.a) flywheels = true;
        if (gamepad1.b) flywheels = false;

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        robot.drivetrain.setMecanumTargets(forward, right, gamepad1.right_stick_x, false);

        double nowS = System.nanoTime() * 1e-9;
        double dtS = Double.isNaN(lastS) ? 0 : nowS - lastS;
        lastS = nowS;

        Pose pose = robot.follower.pose();
        Velocity velocity = robot.follower.velocity();
        double h = pose.heading();
        double forwardInPerS = forward * BettaConstants.foresight.maxAchievableForwardVelocity;
        double leftInPerS = -right * BettaConstants.foresight.maxAchievableStrafeVelocity;
        motion.xIn = pose.x();
        motion.yIn = pose.y();
        motion.headingRad = h;
        boolean moving = aim == Aim.SOTM;
        motion.vxInPerS = moving ? velocity.vx : 0;
        motion.vyInPerS = moving ? velocity.vy : 0;
        motion.omegaRadPerS = moving ? velocity.omega : 0;
        motion.commandVxInPerS = moving ? forwardInPerS * Math.cos(h) - leftInPerS * Math.sin(h) : Double.NaN;
        motion.commandVyInPerS = moving ? forwardInPerS * Math.sin(h) + leftInPerS * Math.cos(h) : Double.NaN;

        solver.setGeometry(POLLEN, ShooterGeometry.pollen());
        solver.setGeometry(NECTAR, ShooterGeometry.nectar());
        solver.setMeasuredRpm(POLLEN, flywheels ? shooter.getPollenRPM() : Double.NaN);
        solver.setMeasuredRpm(NECTAR, flywheels ? shooter.getNectarRPM() : Double.NaN);
        results = solver.solve(motion, selector.forShooters(2), blue,
                FieldConfig.fieldWidthInches, nowS);

        ShotSolver.Result pollen = results[POLLEN];
        ShotSolver.Result nectar = results[NECTAR];
        turret.setSotmAngles(
                pollen.turretDeg + pollen.rateDegPerS * ShotTuning.pollenShooter.servoLagS,
                nectar.turretDeg + nectar.rateDegPerS * ShotTuning.nectarShooter.servoLagS);
        shooter.setSotmTargets(pollen.rpm, pollen.hoodPosition, nectar.rpm, nectar.hoodPosition);
        shooter.setState(flywheels ? Shooter.PollenState.SOTM : Shooter.PollenState.OFF);
        shooter.setState(flywheels ? Shooter.NectarState.SOTM : Shooter.NectarState.OFF);
        shooter.setState(Shooter.PollenHoodState.SOTM);
        shooter.setState(Shooter.NectarHoodState.SOTM);

        updateReady(POLLEN, pollen, ShotTuning.pollenShooter, shooter.getPollenRPM(), dtS);
        updateReady(NECTAR, nectar, ShotTuning.nectarShooter, shooter.getNectarRPM(), dtS);

        // One hardstop feeds both shooters, so a shot waits for every enabled one.
        boolean allReady = (pollenEnabled || nectarEnabled) && (!pollenEnabled || ready[POLLEN]) && (!nectarEnabled || ready[NECTAR]);
        if (allReady) lastAllReadyS = nowS;
        // Keep feeding through a brief dropout, or a burst stutters each time a flywheel dips.
        boolean readyEnough = allReady || (firing && nowS - lastAllReadyS <= ShotTuning.readyDropoutS);
        firing = gamepad1.right_trigger > 0.5 && flywheels && readyEnough;
        shooter.setState(firing ? Shooter.ShooterHardstopState.UNBLOCKED : Shooter.ShooterHardstopState.BLOCKED);
    }

    /** No turret encoder: the turret's angle is estimated as a first-order lag of the command. */
    private void updateReady(int i, ShotSolver.Result r, ShotTuning.ShooterTuning t, double measuredRpm, double dtS) {
        double commanded = r.turretDeg + r.rateDegPerS * t.servoLagS;
        double blend = t.servoLagS > 0 ? 1 - Math.exp(-dtS / t.servoLagS) : 1;
        estimatedTurretDeg[i] += (commanded - estimatedTurretDeg[i]) * blend;
        ready[i] = r.isValid()
                && Math.abs(estimatedTurretDeg[i] - r.turretDeg) <= t.angleToleranceDeg
                && Math.abs(measuredRpm - r.rpm) <= t.rpmTolerance;
    }

    @Override
    protected void telemetry() {
        telemetry.addData("Target CELL", "%s (%s)", selector.upCell(), cellByFieldHalf ? "by field half" : "X toggles");
        telemetry.addData("SOTM", "%s (Y toggles; off is stationary aim), aim %s", ShotTuning.sotmEnabled ? "ON" : "OFF", aim);
        telemetry.addData("Flywheels", "%s (A on, B off), firing %s (hold RT)", flywheels ? "ON" : "OFF", firing);
        if (results == null) return;
        logShooter("Pollen", results[POLLEN], ready[POLLEN]);
        logShooter("Nectar", results[NECTAR], ready[NECTAR]);
    }

    private void logShooter(String name, ShotSolver.Result r, boolean isReady) {
        telemetry.addData(name + " Valid", "%s%s, ready %s", r.isValid(), r.isValid() ? "" : " (" + r.invalid + ")", isReady);
        telemetry.addData(name + " CELL / Element", "%s / %s", r.cell, r.element);
        telemetry.addData(name + " Virtual Target (in)", "(%.1f, %.1f), distance %.1f (%.0f in/s), %d iterations", r.virtualXIn, r.virtualYIn, r.distanceIn, r.distanceRateInPerS, r.iterations);
        telemetry.addData(name + " TOF (s)", "%.3f, effective %.3f", r.tofS, r.effTofS);
        telemetry.addData(name + " Turret (deg)", "%.1f, rate FF %.1f deg/s (analytic %.1f, numerical %.1f)", r.turretDeg, r.rateDegPerS, r.analyticRateDegPerS, r.numericalRateDegPerS);
        telemetry.addData(name + " Flywheel (RPM)", "%.0f (droop %.0f), hood %.3f", r.rpm, r.droopRpm, r.hoodPosition);
        telemetry.addData(name + " Entry (in)", "u %.1f, w %.1f, vz %.0f in/s", r.entryUIn, r.entryWIn, r.entryVzInPerS);
    }
}
