package org.firstinspires.ftc.teamcode.opmodes.test;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.opmodes.test.auto.BallCollectionRobot;

public class MockRobot extends BallCollectionRobot {
    public MockMechanism mech;

    public MockRobot(EnhancedOpMode opMode) {
        super(opMode);
    }

    @Override
    protected void initializeGameModules() {
        mech = new MockMechanism(opMode.hardwareMap);
    }
}
