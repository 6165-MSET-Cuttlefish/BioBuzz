package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.architecture.hardware.photon.Photon;

// @Photon isn't inherited, so Bench: Write Recovery itself keeps the SDK's hubs.
@Photon
@TeleOp(name = "Bench: Write Recovery Photon", group = "Test")
public class WriteRecoveryPhotonBench extends WriteRecoveryBench {
}
