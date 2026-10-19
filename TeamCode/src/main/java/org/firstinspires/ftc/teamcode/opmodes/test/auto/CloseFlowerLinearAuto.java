package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField;

/** Baseline for {@link CloseFlowerAuto}: the same paths on a bare LinearOpMode, Pedro and Ivy, without the framework. */
@Autonomous(name = "Close Flower Linear Auto", group = "Test")
public class CloseFlowerLinearAuto extends BarePedroAuto {

    private CloseFlowerPaths paths;

    @Override
    protected Pose startPose() {
        FieldConfig.setSymmetry(BioBuzzField.SYMMETRY);
        paths = new CloseFlowerPaths();
        return paths.start;
    }

    @Override
    protected Command routine() {
        Path[] legs = paths.all();
        Command[] follows = new Command[legs.length];
        for (int i = 0; i < legs.length; i++) follows[i] = follow(follower, legs[i]);
        return sequential(follows);
    }
}
