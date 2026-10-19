package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

final class BettaBareDrive {
    final GoBildaPinpointDriver pinpoint;
    private final DcMotorEx fl, bl, fr, br;

    BettaBareDrive(HardwareMap hardwareMap, boolean brake) {
        BettaConstants.MecanumSettings m = BettaConstants.mecanum;
        fl = motor(hardwareMap, Drivetrain.FRONT_LEFT, m.frontLeftDirection, brake);
        bl = motor(hardwareMap, Drivetrain.BACK_LEFT, m.backLeftDirection, brake);
        fr = motor(hardwareMap, Drivetrain.FRONT_RIGHT, m.frontRightDirection, brake);
        br = motor(hardwareMap, Drivetrain.BACK_RIGHT, m.backRightDirection, brake);

        // The same calls, in the same order, as Pedro's PinpointLocalizer.
        BettaConstants.PinpointSettings p = BettaConstants.pinpoint;
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, p.name);
        pinpoint.setOffsets(p.xPodOffset, p.yPodOffset, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(p.podType);
        pinpoint.setEncoderDirections(p.xPodDirection, p.yPodDirection);
        pinpoint.resetPosAndIMU();
    }

    private static DcMotorEx motor(HardwareMap hardwareMap, String name, DcMotorSimple.Direction direction, boolean brake) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        if (brake) motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    // Mirrors Drivetrain.setMecanumTargets (robot-centric) so this drives like BioBuzz Tele.
    void drive(Gamepad gamepad) {
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;

        double frontLeft = y + x + rx;
        double backLeft = y - x + rx;
        double frontRight = y - x - rx;
        double backRight = y + x - rx;

        double max = Math.max(1.0, Math.max(Math.abs(frontLeft), Math.max(Math.abs(backLeft),
                Math.max(Math.abs(frontRight), Math.abs(backRight)))));

        fl.setPower(frontLeft / max);
        bl.setPower(backLeft / max);
        fr.setPower(frontRight / max);
        br.setPower(backRight / max);
    }

    void stop() {
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }
}
