package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzOpMode;
import org.firstinspires.ftc.teamcode.biobuzz.modules.Shooter;

@TeleOp(name = "BioBuzz Shooter Test", group = "Test")
public class BioBuzzShooterTest extends BioBuzzOpMode {

    private Shooter shooter;

    @Override
    protected void initialize() {
        disableBallVision();
        shooter = new Shooter(hardwareMap);
    }

    @Override
    protected void gameLoop() {
        if (gamepad1.a) shooter.setState(Shooter.PollenState.ON);
        if (gamepad1.b) shooter.setState(Shooter.PollenState.OFF);
        if (gamepad1.left_bumper) shooter.setState(Shooter.PollenState.OPEN_LOOP);
        if (gamepad1.x) shooter.setState(Shooter.NectarState.ON);
        if (gamepad1.y) shooter.setState(Shooter.NectarState.OFF);
        if (gamepad1.right_bumper) shooter.setState(Shooter.NectarState.OPEN_LOOP);
    }

    @Override
    protected void telemetry() {
        telemetry.addData("Controls", "A/B: pollen on/off, X/Y: nectar on/off, LB/RB: pollen/nectar open loop");
    }
}
