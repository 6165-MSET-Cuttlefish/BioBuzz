package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzOpMode;

@TeleOp(name = "BioBuzz Tele", group = "A")
public class BioBuzzTele extends BioBuzzOpMode {

    @Override
    protected void gameLoop() {
        robot.drivetrain.setMecanumTargets(
                -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false);
    }
}
