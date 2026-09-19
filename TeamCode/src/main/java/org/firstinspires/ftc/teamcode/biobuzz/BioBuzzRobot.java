package org.firstinspires.ftc.teamcode.biobuzz;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

public class BioBuzzRobot extends Robot {
    public Drivetrain drivetrain;

    public BioBuzzRobot(EnhancedOpMode opMode) throws InterruptedException {
        super(opMode);
    }

    @Override
    protected Follower createFollower(HardwareMap hardwareMap) {
        return BettaConstants.create(hardwareMap);
    }

    @Override
    protected void initializeGameModules() {
        drivetrain = new Drivetrain(opMode.hardwareMap).withFollower(follower);
    }
}
