package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;

@TeleOp(name = "Bench: Gamepad", group = "Test")
public class GamepadBench extends EnhancedOpMode {
    private BenchRobot bench;
    private BenchReport report;
    private GamepadProbe probe;

    @Override
    protected Robot createRobot() {
        bench = new BenchRobot(this);
        return bench;
    }

    @Override
    protected void initialize() {
        report = new BenchReport("Gamepad", "gamepad");
        bench.drivetrain.setWriteEnabled(false);
        probe = new GamepadProbe(report, gamepad1, gamepad2, "iterative");
    }

    @Override
    protected void initializeLoop() {
        probe.sample(false);
    }

    @Override
    protected void gameLoop() {
        probe.sample(true);
        bench.drivetrain.setMecanumTargets(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false);
    }

    @Override
    protected void telemetry() {
        report.addTo(telemetry);
        probe.put(telemetry::addData);
    }
}
