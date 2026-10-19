package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Pinpoint Drive Test", group = "Test")
public class PinpointDriveTest extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BettaBareDrive drive = new BettaBareDrive(hardwareMap, true);
        pinpoint = drive.pinpoint;

        while (opModeInInit()) update();
        while (opModeIsActive()) {
            drive.drive(gamepad1);
            update();
        }

        drive.stop();
    }

    private void update() {
        pinpoint.update();
        telemetry.addData("x (in)", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("y (in)", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("heading (deg)", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        telemetry.update();
    }
}
