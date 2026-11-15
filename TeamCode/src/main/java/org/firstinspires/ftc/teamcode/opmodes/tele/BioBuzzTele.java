package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.architecture.input.GamepadPresses;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzOpMode;
import org.firstinspires.ftc.teamcode.biobuzz.modules.CellTipCamera;

@Config("BioBuzz Tele")
@TeleOp(name = "BioBuzz Tele", group = "A")
public class BioBuzzTele extends BioBuzzOpMode {

    public static boolean cellTip = true;

    private static final double SLOW_MULTIPLIER = 0.75;

    private ActionLayer d1Layer = ActionLayer.TELE;
    private ActionLayer d2Layer = ActionLayer.TELE;
    private double slowMultiplier = 1.0;

    @Override
    protected void initialize() {
        disableBallVision();
        applyCellTipFlag();
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
        drive();
    }

    private void setBothLayers(ActionLayer layer) {
        setD1Layer(layer);
        setD2Layer(layer);
    }

    private void setD1Layer(ActionLayer layer) {
        if (layer == d1Layer) return;
        d1Layer = layer;
        GamepadPresses.clear(gamepad1);
    }

    private void setD2Layer(ActionLayer layer) {
        if (layer == d2Layer) return;
        d2Layer = layer;
        GamepadPresses.clear(gamepad2);
    }

    private void updateInputs() {
        switch (d1Layer) {
            case TELE:
                d1TeleControls();
                break;
        }
        switch (d2Layer) {
            case TELE:
                d2TeleControls();
                break;
        }
    }

    private void d1TeleControls() {
        if (gamepad1.leftTriggerWasPressed()) {
            slowMultiplier = slowMultiplier == SLOW_MULTIPLIER ? 1.0 : SLOW_MULTIPLIER;
        }
        if (gamepad1.leftBumperWasPressed()) {
            if (robot.drivetrain.isHeadingLocked()) robot.drivetrain.unlockHeading();
            else robot.drivetrain.lockHeading();
        }
    }

    private void d2TeleControls() {
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
        telemetry.addData("Heading Lock", robot.drivetrain.isHeadingLocked() ? "ON" : "OFF");
    }
}
