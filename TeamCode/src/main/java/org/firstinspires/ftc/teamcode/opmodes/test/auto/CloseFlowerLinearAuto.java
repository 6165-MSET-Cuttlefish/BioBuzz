package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

/** Baseline for {@link CloseFlowerAuto}: the same paths on a bare LinearOpMode, Pedro and Ivy, without the framework. */
@Autonomous(name = "Close Flower Linear Auto", group = "Test")
public class CloseFlowerLinearAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Scheduler.reset();
        FieldConfig.setSymmetry(BioBuzzField.SYMMETRY);
        Follower follower = BettaConstants.create(hardwareMap);
        CloseFlowerPaths paths = new CloseFlowerPaths();
        follower.setPose(paths.start);
        follower.update();

        Path[] legs = paths.all();
        Command[] follows = new Command[legs.length];
        for (int i = 0; i < legs.length; i++) follows[i] = follow(follower, legs[i]);
        Command routine = sequential(follows);

        waitForStart();
        schedule(routine);

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }
}
