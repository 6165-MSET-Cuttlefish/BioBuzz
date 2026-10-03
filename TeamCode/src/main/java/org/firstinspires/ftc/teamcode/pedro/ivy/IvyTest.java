package org.firstinspires.ftc.teamcode.pedro.ivy;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;

@Config
@TeleOp
public class IvyTest extends LinearOpMode {
    public static boolean end = false;
    public static boolean a = false, b = false;
    public static boolean inc = false;
    public int num = 0;
    @Override
    public void runOpMode() {
        Scheduler.reset();

        DcMotor leftShooter = hardwareMap.get(DcMotor.class, "leftFlywheel");
        DcMotor rightShooter = hardwareMap.get(DcMotor.class, "rightFlywheel");

        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Command spinUpHigh = Command.build()
                .setStart(() -> a = false)
                .setExecute(() -> {
                    leftShooter.setPower(0.8);
                    rightShooter.setPower(0.8);
                })
                .setPriority(1)
                .requiring(leftShooter, rightShooter);

        Command spinUpLow = Command.build()
                .setStart(() -> b = false)
                .setExecute(() -> {
                    leftShooter.setPower(0.3);
                    rightShooter.setPower(0.3);
                })
                .setPriority(2)
                .requiring(leftShooter, rightShooter);

        Command stop = Command.build()
                .setStart(() -> end = false)
                .setExecute(() -> {
                    leftShooter.setPower(0);
                    rightShooter.setPower(0);
                })
                .setPriority(3)
                .requiring(leftShooter, rightShooter);

        Command increment = Command.build()
                .setStart(() -> inc = false)
                .setExecute(() -> num++)
                .setDone(() -> true);

        waitForStart();

        while (opModeIsActive()) {
            Scheduler.execute();

            if (a) schedule(spinUpLow);
            if (b) schedule(spinUpHigh);
            if (end) schedule(stop);
            if (inc) schedule(increment);

            telemetry.addData("leftPower", leftShooter.getPower());
            telemetry.addData("rightPower", rightShooter.getPower());
            telemetry.update();
        }
    }
}