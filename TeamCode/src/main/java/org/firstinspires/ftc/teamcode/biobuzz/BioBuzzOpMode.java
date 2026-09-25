package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;

public abstract class BioBuzzOpMode extends EnhancedOpMode {

    protected BioBuzzRobot robot;

    @Override
    protected final Robot createRobot() {
        robot = new BioBuzzRobot(this); // shadows EnhancedOpMode.robot with a typed reference to the same object
        return robot;
    }
}
