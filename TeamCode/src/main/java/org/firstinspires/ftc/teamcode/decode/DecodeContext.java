package org.firstinspires.ftc.teamcode.decode;

import static org.firstinspires.ftc.teamcode.decode.modules.Turret.turretX;
import static org.firstinspires.ftc.teamcode.decode.modules.Turret.turretY;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

@Config("DecodeContext")
public final class DecodeContext {

    public static final Pose redTargetPose = new Pose(141.5 - 17, 144);
    public static final Pose blueTargetPose = new Pose(17, 144);

    public static double turretFieldX = 0;
    public static double turretFieldY = 0;

    /** Aim point, corrected for robot motion when SOTM is on; not the raw goal pose. */
    public static double targetX = 0;
    public static double targetY = 0;

    public static double distanceToGoal = 0;

    public static boolean sotmVelocity = true;
    public static boolean sotmAngle = false;
    public static boolean sotmAccel = true;
    public static double sotmAccelScale = 50;
    public static double sotmDragScale = 0.3;

    /** Runs before module reads and before follower.update(), so it uses last loop's pose and velocity. */
    public static void updateSharedPose(DecodeRobot robot) {
        Pose robotPose = robot.follower.pose();
        double robotRad = robotPose.heading();

        Pose turretField = turretFieldPosition(robotPose);
        turretFieldX = turretField.x();
        turretFieldY = turretField.y();

        double targetX = robot.targetPose.x();
        double targetY = robot.targetPose.y();

        if (sotmVelocity) {
            double currentDistance = Math.hypot(targetX - turretFieldX, targetY - turretFieldY);
            robot.turret.flightTime = 0.00103923 * currentDistance + 0.528024;

            Velocity robotVelocity = robot.follower.velocity();
            double robotVx = robotVelocity.vx;
            double robotVy = robotVelocity.vy;

            targetX -= robotVx * robot.turret.flightTime;
            targetY -= robotVy * robot.turret.flightTime;

            if (sotmAccel) {
                // [fl, bl, br, fr]; localThrustY is always 0 for mecanum commands, kept because sotmAccelScale was tuned with it.
                double[] powers = robot.drivetrain.getMotorPowers();
                double localThrustX = (powers[0] + powers[1] + powers[2] + powers[3]) / 4.0;
                double localThrustY = (powers[0] - powers[1] - powers[2] + powers[3]) / 4.0;

                double cos = Math.cos(robotRad);
                double sin = Math.sin(robotRad);

                double thrustX = localThrustX * cos - localThrustY * sin;
                double thrustY = localThrustX * sin + localThrustY * cos;

                double accelX = thrustX * sotmAccelScale - robotVx * sotmDragScale;
                double accelY = thrustY * sotmAccelScale - robotVy * sotmDragScale;

                double t = robot.turret.flightTime;
                targetX -= 0.5 * accelX * t * t;
                targetY -= 0.5 * accelY * t * t;
            }

            if (sotmAngle) {
                double angleCompensation = -robotVelocity.omega * robot.turret.flightTime;

                double dx = targetX - turretFieldX;
                double dy = targetY - turretFieldY;

                double cosA = Math.cos(angleCompensation);
                double sinA = Math.sin(angleCompensation);

                targetX = turretFieldX + dx * cosA - dy * sinA;
                targetY = turretFieldY + dx * sinA + dy * cosA;
            }
        }

        DecodeContext.targetX = targetX;
        DecodeContext.targetY = targetY;
        distanceToGoal = Math.hypot(targetX - turretFieldX, targetY - turretFieldY);
    }

    /** Turret pivot's field x/y for a robot at {@code robot}; the returned heading is unused. */
    public static Pose turretFieldPosition(Pose robot) {
        double h = robot.heading();
        return new Pose(
                robot.x() + turretX * Math.cos(h) - turretY * Math.sin(h),
                robot.y() + turretX * Math.sin(h) + turretY * Math.cos(h));
    }

    private DecodeContext() {}
}
