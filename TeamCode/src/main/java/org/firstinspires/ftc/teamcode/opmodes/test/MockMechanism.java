package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;

/**
 * Mock module for the architecture smoke test. Blank {@link Tuning#pulseMotorName} = no hardware
 * (safe default); never name a drivetrain motor (fl/bl/fr/br) — it fights the Pedro follower.
 */
public class MockMechanism extends Module {
    public enum Status implements State {
        IDLE(0), ACTIVE(1);
        Status(double value) { setValue(value); }
    }

    @Config("Mock Mechanism")
    public static class Tuning {
        public static String pulseMotorName = "";
        public static double pulsePower = 0.2;
    }

    private final HardwareMap hardwareMap;
    private EnhancedMotor motor;
    private String boundName = "";

    public MockMechanism(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override protected void initStates() { setStates(Status.IDLE); }

    @Override
    protected void read() {
        String name = Tuning.pulseMotorName == null ? "" : Tuning.pulseMotorName.trim();
        if (!name.equals(boundName)) {
            boundName = name;
            // Safe the old motor before dropping it; write()/stop() only touch the current one.
            if (motor != null) motor.setPower(0.0);
            motor = null;
            // boundName is updated first so an unknown name throws once (fail-fast), not every loop.
            if (!name.isEmpty()) {
                motor = new EnhancedMotor(hardwareMap, name);
            }
        }
    }

    @Override
    protected void write() {
        if (motor != null) motor.setPower(isInAny(Status.ACTIVE) ? Tuning.pulsePower : 0.0);
    }

    @Override
    public void stop() {
        if (motor != null) motor.setPower(0.0);
    }

    public boolean isMotorBound() { return motor != null; }
    public String boundMotorName() { return boundName.isEmpty() ? "(none)" : boundName; }
}
