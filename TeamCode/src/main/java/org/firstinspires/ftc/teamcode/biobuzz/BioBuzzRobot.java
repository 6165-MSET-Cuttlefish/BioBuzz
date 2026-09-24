package org.firstinspires.ftc.teamcode.biobuzz;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

public class BioBuzzRobot extends Robot {
    public Drivetrain drivetrain;
    public LimelightCamera limelight;
    public RobotActions actions;

    public BioBuzzRobot(EnhancedOpMode opMode) {
        super(opMode);
    }

    @Override
    protected Follower createFollower(HardwareMap hardwareMap) {
        return BettaConstants.create(hardwareMap);
    }

    @Override
    protected void initializeGameModules() {
        drivetrain = new Drivetrain(opMode.hardwareMap).withFollower(follower);
        limelight = new LimelightCamera(opMode.hardwareMap);
        actions = new RobotActions(this);
    }
}
