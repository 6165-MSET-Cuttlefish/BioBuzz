package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldPose;
import org.firstinspires.ftc.teamcode.architecture.auto.Region;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

public class BallCollectionRobot extends Robot {
    public BallCollectionRobot(EnhancedOpMode opMode) {
        super(opMode);
        FieldConfig.setSymmetry(BioBuzzField.SYMMETRY);
    }

    @Override
    protected Follower createFollower(HardwareMap hardwareMap) {
        return BettaConstants.create(hardwareMap);
    }

    @Override
    protected void initializeGameModules() {}

    /**
     * This alliance's half of the field, {@code marginIn} in from the walls and the centre line, for the robot's
     * centre: RED's is x in [m, width/2 - m], BLUE's (through {@link FieldPose}) x in [width/2 + m, width - m].
     */
    static Region ownHalf(double marginIn) {
        double width = FieldConfig.fieldWidthInches;
        if (!(marginIn >= 0 && 4 * marginIn < width)) {
            throw new IllegalArgumentException(String.format(
                    "wall margin %.1f in leaves no room in half of a %.1f in field", marginIn, width));
        }
        Pose a = FieldPose.forAlliance(marginIn, marginIn, 0);
        Pose b = FieldPose.forAlliance(width / 2 - marginIn, width - marginIn, 0);
        return Region.spanning(a, b);
    }
}
