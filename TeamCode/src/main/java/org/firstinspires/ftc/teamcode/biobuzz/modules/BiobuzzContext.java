package org.firstinspires.ftc.teamcode.biobuzz.modules;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;

@Config("Biobuzz Context")
public final class BiobuzzContext {

    //front is defined as the side closer to the audience
    //from the audience point of view, the red alliance is on the left, and the blue alliance is on the right
    public static final Pose RedHiveFront = new Pose(0, 0, 0);
    public static final Pose RedHiveBack = new Pose(0, 0, 0);
    public static final Pose BlueHiveFront = new Pose(0, 0, 0);
    public static final Pose BlueHiveBack = new Pose(0, 0, 0);


    public static Pose getHive(Pose pose) {
        if (pose.y() < 72) {
            if (Context.allianceColor == AllianceColor.BLUE) {
                return BlueHiveFront;
            } else if (Context.allianceColor == AllianceColor.RED) {
                return RedHiveFront;
            }
        } else {
            if (Context.allianceColor == AllianceColor.BLUE) {
                return BlueHiveBack;
            } else if (Context.allianceColor == AllianceColor.RED) {
                return RedHiveBack;
            }
        }
        return null;
    }

}
