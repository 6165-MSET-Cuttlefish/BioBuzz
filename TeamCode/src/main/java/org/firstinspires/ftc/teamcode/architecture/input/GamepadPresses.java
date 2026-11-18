package org.firstinspires.ftc.teamcode.architecture.input;

import com.qualcomm.robotcore.hardware.Gamepad;

public final class GamepadPresses {
    private GamepadPresses() {}

    // Reads every flag rather than calling resetEdgeDetection(), which would turn a held button into a fresh press.
    public static void clear(Gamepad g) {
        g.dpadUpWasPressed();
        g.dpadUpWasReleased();
        g.dpadDownWasPressed();
        g.dpadDownWasReleased();
        g.dpadLeftWasPressed();
        g.dpadLeftWasReleased();
        g.dpadRightWasPressed();
        g.dpadRightWasReleased();
        g.aWasPressed();
        g.aWasReleased();
        g.bWasPressed();
        g.bWasReleased();
        g.xWasPressed();
        g.xWasReleased();
        g.yWasPressed();
        g.yWasReleased();
        g.crossWasPressed();
        g.crossWasReleased();
        g.circleWasPressed();
        g.circleWasReleased();
        g.squareWasPressed();
        g.squareWasReleased();
        g.triangleWasPressed();
        g.triangleWasReleased();
        g.guideWasPressed();
        g.guideWasReleased();
        g.psWasPressed();
        g.psWasReleased();
        g.startWasPressed();
        g.startWasReleased();
        g.optionsWasPressed();
        g.optionsWasReleased();
        g.backWasPressed();
        g.backWasReleased();
        g.shareWasPressed();
        g.shareWasReleased();
        g.leftBumperWasPressed();
        g.leftBumperWasReleased();
        g.rightBumperWasPressed();
        g.rightBumperWasReleased();
        g.leftStickButtonWasPressed();
        g.leftStickButtonWasReleased();
        g.rightStickButtonWasPressed();
        g.rightStickButtonWasReleased();
        g.touchpadWasPressed();
        g.touchpadWasReleased();
        g.leftTriggerWasPressed();
        g.leftTriggerWasReleased();
        g.rightTriggerWasPressed();
        g.rightTriggerWasReleased();
    }
}
