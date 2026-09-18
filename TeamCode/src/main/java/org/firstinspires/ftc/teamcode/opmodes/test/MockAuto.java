package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.architecture.auto.PathCommands;
import org.firstinspires.ftc.teamcode.architecture.command.StateCommands;
import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;

/**
 * End-to-end smoke test for the framework: lifecycle, Robot/Module/State, Ivy commands, telemetry.
 * Enable {@code Tuning.enableDrive} only with wheels off the ground and motor + odometry directions
 * confirmed — the Foresight gains in {@code pedro.Constants} are untuned placeholders.
 */
@Autonomous(name = "Mock Architecture Test", group = "test")
public class MockAuto extends EnhancedOpMode {

    @Config("Mock Auto")
    public static class Tuning {
        public static boolean enableDrive = false;
        public static double driveInches = 24;
        public static int safetyTimeoutMs = 15000;
    }

    private String phase = "init";
    private MockRobot mock;
    private Command sequence;

    @Override
    protected Robot createRobot() throws InterruptedException {
        Context.allianceColor = AllianceColor.RED;
        mock = new MockRobot(this);
        return mock;
    }

    @Override
    protected void initialize() {
        Pose start = robot.follower.pose();
        double heading = start.heading();

        List<Command> steps = new ArrayList<>();
        steps.add(StateCommands.set(MockMechanism.Status.ACTIVE));
        steps.add(instant(() -> phase = "sequence started, mech ACTIVE"));

        if (Tuning.enableDrive) {
            Pose forward = new Pose(
                    start.x() + Math.cos(heading) * Tuning.driveInches,
                    start.y() + Math.sin(heading) * Tuning.driveInches,
                    heading);
            steps.add(instant(() -> phase = "driving forward"));
            steps.add(PathCommands.follow(robot.follower, Paths.line(start, forward).constant(heading)));
            steps.add(instant(() -> phase = "pause at far point"));
            steps.add(waitMs(750));
            steps.add(instant(() -> phase = "driving back"));
            steps.add(PathCommands.follow(robot.follower, Paths.line(forward, start).constant(heading)));
        } else {
            steps.add(instant(() -> phase = "blocking wait (no-motion mode)"));
            steps.add(waitMs(1500));
        }

        steps.add(StateCommands.set(MockMechanism.Status.IDLE));
        steps.add(instant(() -> phase = "sequence complete"));

        // The timeout branch ends NATURALLY when it fires and INTERRUPTED when the sequence finishes first.
        sequence = race(
                sequential(steps.toArray(new Command[0])),
                waitMs(Tuning.safetyTimeoutMs).setEnd(end -> {
                    if (end == EndCondition.NATURALLY) {
                        phase = "SAFETY TIMEOUT";
                        robot.follower.stop();
                    }
                }));
    }

    // start() resets the scheduler, so the sequence is scheduled here rather than where it is built.
    @Override
    protected void onStart() {
        Scheduler.schedule(sequence);
    }

    @Override
    protected void telemetry() {
        telemetry.addData("Mock Phase", phase);
        telemetry.addData("Drive Enabled", Tuning.enableDrive);
        telemetry.addData("Pulse Motor", mock.mech.boundMotorName()
                + (mock.mech.isMotorBound() ? " [bound]" : " [none — set MockMechanism.pulseMotorName]"));
        telemetry.addData("Running", Scheduler.isRunning(sequence));
    }
}
