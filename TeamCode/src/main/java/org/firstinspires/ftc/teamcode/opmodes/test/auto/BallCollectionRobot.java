package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
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
}
