package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.architecture.input.EdgeBooleanSupplier;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzOpMode;

@TeleOp(name = "BioBuzz Tele", group = "A")
public class BioBuzzTele extends BioBuzzOpMode {

    private static final double SLOW_MULTIPLIER = 0.75;

    private EdgeBooleanSupplier slowToggle;
    private EdgeBooleanSupplier headingLockToggle;
    private boolean slowMode;

    @Override
    protected void initialize() {
        slowToggle = new EdgeBooleanSupplier(() -> gamepad1.left_trigger > 0.1);
        headingLockToggle = new EdgeBooleanSupplier(() -> gamepad1.left_bumper);
    }

    @Override
    protected void onStart() {
        for (EnhancedMotor motor : new EnhancedMotor[] {robot.drivetrain.getFl(), robot.drivetrain.getBl(),
                robot.drivetrain.getBr(), robot.drivetrain.getFr()}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    @Override
    protected void gameLoop() {
        if (slowToggle.wasJustPressed()) slowMode = !slowMode;
        if (headingLockToggle.wasJustPressed()) {
            if (robot.drivetrain.isHeadingLocked()) robot.drivetrain.unlockHeading();
            else robot.drivetrain.lockHeading();
        }

        double scale = slowMode ? SLOW_MULTIPLIER : 1.0;
        robot.drivetrain.setMecanumTargets(-gamepad1.left_stick_y * scale, gamepad1.left_stick_x * scale,
                gamepad1.right_stick_x * scale, false);
    }

    @Override
    protected void telemetry() {
        telemetry.addData("Slow Mode", slowMode ? "75%" : "OFF");
        telemetry.addData("Heading Lock", robot.drivetrain.isHeadingLocked() ? "ON" : "OFF");
    }
}
