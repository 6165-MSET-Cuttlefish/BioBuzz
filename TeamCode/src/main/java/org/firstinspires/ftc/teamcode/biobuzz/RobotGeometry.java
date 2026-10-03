package org.firstinspires.ftc.teamcode.biobuzz;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.architecture.auto.RobotShape;

/**
 * The robot's footprint and intake as the ball planner sees them, in inches: the Betta bot, the test robot until the
 * BIOBUZZ robot is built, 16 in long and 15 in wide with the middle of its intake on the front face, 8 in ahead of the
 * centre of rotation.
 */
@Config("Robot Geometry")
public final class RobotGeometry {
    private RobotGeometry() {}

    /** Along the heading. */
    public static double lengthIn = 16;
    /** Across the heading. */
    public static double widthIn = 15;
    /** From the centre of rotation forward to the middle of the intake; past lengthIn / 2 the intake sticks out, and the planner keeps it clear too. */
    public static double intakeOffsetIn = 8;
    /** Wider than widthIn, the intake sticks out at the sides, and the planner keeps it clear too. */
    public static double intakeWidthIn = 15;

    public static RobotShape shape() {
        return new RobotShape(lengthIn, widthIn, intakeOffsetIn, intakeWidthIn);
    }
}
