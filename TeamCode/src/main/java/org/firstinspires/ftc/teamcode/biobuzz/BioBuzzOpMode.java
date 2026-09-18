package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;

/** Base for OpModes driving the {@link BioBuzzRobot}; adds a typed {@code robot} so game code needn't cast. */
public abstract class BioBuzzOpMode extends EnhancedOpMode {

    protected BioBuzzRobot robot;

    @Override
    protected final Robot createRobot() throws InterruptedException {
        BioBuzzRobot r = new BioBuzzRobot(this);
        this.robot = r; // shadows EnhancedOpMode.robot with a typed reference to the same object
        return r;
    }
}
