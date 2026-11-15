package org.firstinspires.ftc.teamcode.biobuzz;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.biobuzz.modules.Camera;
import org.firstinspires.ftc.teamcode.biobuzz.modules.CellTipCamera;
import org.firstinspires.ftc.teamcode.biobuzz.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.pedro.BioBuzzConstants;

public class BioBuzzRobot extends Robot {
    public Drivetrain drivetrain;
    public CellTipCamera cellTip;
    public Camera camera;
    public RobotActions actions;

    public BioBuzzRobot(EnhancedOpMode opMode) {
        super(opMode);
        BioBuzzField.configure();
    }

    @Override
    protected Follower createFollower(HardwareMap hardwareMap) {
        return BioBuzzConstants.create(opMode);
    }

    @Override
    protected void initializeGameModules() {
        drivetrain = new Drivetrain(opMode.hardwareMap, follower);
        cellTip = new CellTipCamera(opMode.hardwareMap);
        camera = new Camera(opMode.hardwareMap, follower);
        actions = new RobotActions(this);
    }
}
