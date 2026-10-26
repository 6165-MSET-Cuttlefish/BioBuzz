package org.firstinspires.ftc.teamcode.biobuzz.modules;

import android.support.v4.app.INotificationSideChannel;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedServo;
import org.firstinspires.ftc.teamcode.decode.modules.Magazine;

public class Intran extends Module {
    private final EnhancedMotor intake;
    private final EnhancedMotor transfer;
    private final EnhancedServo ruler;

    private final AnalogInput sensorLeft, sensorRight;

    private double intakePower, transferPower;
    private double rulerPos;

    public Intran(HardwareMap hardwareMap) {
        intake = new EnhancedMotor(hardwareMap, "intake");
        transfer = new EnhancedMotor(hardwareMap, "transfer");
        ruler = new EnhancedServo(hardwareMap, "ruler");

        sensorLeft = hardwareMap.analogInput.get("sensorLeft");
        sensorRight = hardwareMap.analogInput.get("sensorRight");

        transfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public enum RulerState implements State {
        UP(1.0),
        DOWN(0.0);
        RulerState(double value) {setValue(value);}
    }

    public enum IntakeState implements State {
        FORWARD(1.0),
        SHOOTING(1.0),
        REVERSE(-1.0),
        OFF(0.0);
        IntakeState(double value) {setValue(value);}
    }

    public enum TransferState implements State {
        FORWARD(1.0),
        SHOOTING(1.0),
        REVERSE(-1.0),
        OFF(0.0);
        TransferState(double value) {setValue(value);}
    }


    @Override
    protected void initStates() {
        setStates(RulerState.UP, IntakeState.OFF, TransferState.OFF);
    }

    @Override
    protected void read() {
        intakePower = getState(Intran.IntakeState.class).getValue();
        transferPower = getState(Intran.TransferState.class).getValue();
        rulerPos = getState(Intran.RulerState.class).getValue();
    }

    @Override
    protected void write() {
        intake.setPower(intakePower);
        transfer.setPower(transferPower);
        ruler.setPosition(rulerPos);
    }

    @Override
    public void stop() {
        intake.stop();
        transfer.stop();
    }

    public double getIntakeCurrent() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }

    public double getTransferCurrent() {
        return transfer.getCurrent(CurrentUnit.AMPS);
    }

    //TODO: Add if statements for selective telemetry once central robot class made
    @Override
    protected void onTelemetry() {
        logDashboard("Intake State", getState(Intran.IntakeState.class));
        logDashboard("Intake Power", "%.3f", intakePower);

        logDashboard("Transfer State", getState(Intran.TransferState.class));
        logDashboard("Transfer Power", "%.3f", transferPower);

        logDashboard("Ruler State", getState(Intran.RulerState.class));
        logDashboard("Ruler Pos", "%.3f", rulerPos);

        logDashboard("Intake Current (A)", "%.2f", getIntakeCurrent());
        logDashboard("Transfer Current (A)", "%.2f", getTransferCurrent());
    }
}
