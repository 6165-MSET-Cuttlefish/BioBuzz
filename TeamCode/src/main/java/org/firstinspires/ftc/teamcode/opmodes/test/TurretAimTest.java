package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzOpMode;
import org.firstinspires.ftc.teamcode.modules.Turret;

@TeleOp(name = "Turret Aim Test", group = "Test")
@Config("Turret Aim Test")
public class TurretAimTest extends BioBuzzOpMode {

    public static double startX = 72;
    public static double startY = 72;
    public static double startHeadingDeg = 0;
    public static double goalX = 24;
    public static double goalY = 72;

    private Turret turret;

    @Override
    protected void initialize() {
        disableBallVision();
        robot.follower.setPose(new Pose(startX, startY, Math.toRadians(startHeadingDeg)));
        turret = new Turret(hardwareMap, robot.follower);
        turret.setGoal(() -> new Pose(goalX, goalY));
    }

    @Override
    protected void gameLoop() {
        if (gamepad1.dpad_left) turret.setState(Turret.TurretState.LEFT);
        if (gamepad1.dpad_down) turret.setState(Turret.TurretState.BACK);
        if (gamepad1.dpad_right) turret.setState(Turret.TurretState.RIGHT);
        if (gamepad1.a) turret.setState(Turret.TurretState.AUTOAIM);
        if (gamepad1.b) turret.setState(Turret.TurretState.HOLD);
    }

    @Override
    protected void telemetry() {
        Pose pose = robot.follower.pose();
        telemetry.addData("Robot", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Goal", "(%.1f, %.1f)", goalX, goalY);
        telemetry.addData("Controls", "dpad left/down/right: LEFT/BACK/RIGHT, A: AUTOAIM, B: HOLD");
    }
}
