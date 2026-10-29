package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.hardware.photon.Photon;

// @Photon isn't inherited, so Bench: Framework Loop itself keeps the SDK's hubs.
@Photon
@TeleOp(name = "Bench: Framework Loop Photon", group = "Test")
public class FrameworkLoopPhotonBench extends FrameworkLoopBench {
}
