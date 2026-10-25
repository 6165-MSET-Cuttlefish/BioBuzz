package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzOpMode;
import org.firstinspires.ftc.teamcode.biobuzz.modules.Shooter;

@TeleOp(name = "BioBuzz Shooter Test", group = "Test")
public class BioBuzzShooterTest extends BioBuzzOpMode {

    private Shooter shooter;

    @Override
    protected void initialize() {
        shooter = new Shooter(hardwareMap);
    }

    @Override
    protected void gameLoop() {
        if (gamepad1.a) shooter.setState(Shooter.PollenState.ON);
        if (gamepad1.b) shooter.setState(Shooter.PollenState.OFF);
        if (gamepad1.x) shooter.setState(Shooter.NectarState.ON);
        if (gamepad1.y) shooter.setState(Shooter.NectarState.OFF);
    }

}
