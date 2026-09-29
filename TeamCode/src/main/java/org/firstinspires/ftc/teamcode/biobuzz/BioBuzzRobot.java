package org.firstinspires.ftc.teamcode.biobuzz;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.modules.Camera;
import org.firstinspires.ftc.teamcode.modules.CellTipCamera;
import org.firstinspires.ftc.teamcode.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

public class BioBuzzRobot extends Robot {
    public Drivetrain drivetrain;
    public CellTipCamera cellTip;
    /** Limelight ball vision, field-relative through the follower. */
    public Camera camera;
    public RobotActions actions;

    public BioBuzzRobot(EnhancedOpMode opMode) {
        super(opMode);
        FieldConfig.setSymmetry(BioBuzzField.SYMMETRY);
    }

    @Override
    protected Follower createFollower(HardwareMap hardwareMap) {
        return BettaConstants.create(hardwareMap);
    }

    @Override
    protected void initializeGameModules() {
        drivetrain = new Drivetrain(opMode.hardwareMap).withFollower(follower);
        cellTip = new CellTipCamera(opMode.hardwareMap);
        camera = new Camera(opMode.hardwareMap).withFollower(follower);
        actions = new RobotActions(this);
    }
}
