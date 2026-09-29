package org.firstinspires.ftc.teamcode.decode;

import static org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization.toField;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.decode.modules.Turret;

/** Base for OpModes driving the {@link DecodeRobot}; adds a typed {@code robot} so game code needn't cast. */
public abstract class DecodeOpMode extends EnhancedOpMode {

    protected DecodeRobot robot;

    @Override
    protected final Robot createRobot() {
        DecodeRobot r = new DecodeRobot(this);
        this.robot = r;
        return r;
    }

    @Override
    protected void onLoopStart() {
        robot.updateWriteToggles();
        DecodeContext.updateSharedPose(robot);
    }

    @Override
    protected void dashboardOverlay(Canvas overlay) {
        if (robot.turret.isInAny(Turret.TurretState.AUTOAIM)) {
            Pose turretDash = toField(new Pose(DecodeContext.turretFieldX, DecodeContext.turretFieldY));
            overlay.setStroke("#FFFFFF")
                    .fillCircle(turretDash.x(), turretDash.y(), 2);

            Pose targetDash = toField(new Pose(DecodeContext.targetX, DecodeContext.targetY));
            overlay.setStroke(Context.allianceColor == AllianceColor.BLUE ? "blue" : "red")
                    .fillCircle(targetDash.x(), targetDash.y(), 2);
        }
    }
}
