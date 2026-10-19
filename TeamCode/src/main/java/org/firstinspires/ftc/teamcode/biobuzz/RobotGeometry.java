package org.firstinspires.ftc.teamcode.biobuzz;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.architecture.planner.RobotShape;

@Config("Robot Geometry")
public final class RobotGeometry {
    private RobotGeometry() {}

    public static double lengthIn = 16;
    public static double widthIn = 15;
    public static double intakeOffsetIn = 8;
    public static double intakeWidthIn = 15;

    public static RobotShape shape() {
        return new RobotShape(lengthIn, widthIn, intakeOffsetIn, intakeWidthIn);
    }
}
