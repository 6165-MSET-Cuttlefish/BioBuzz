package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.input.InputClock;

@TeleOp(name = "Bench: Gamepad Linear", group = "Test")
public class GamepadLinearBench extends LinearOpMode {

    @Override
    public void runOpMode() {
        BenchReport report = new BenchReport("Gamepad Linear", "gamepad_linear");
        GamepadProbe probe = new GamepadProbe(report, gamepad1, gamepad2, "linear");
        while (opModeInInit()) {
            poll(probe, false);
            idle();
        }
        while (opModeIsActive()) poll(probe, true);
    }

    private static void poll(GamepadProbe probe, boolean running) {
        InputClock.advance();
        probe.sample(running);
        probe.sendIfDue();
    }
}
