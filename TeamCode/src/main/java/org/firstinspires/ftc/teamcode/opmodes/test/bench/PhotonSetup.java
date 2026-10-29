package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import org.firstinspires.ftc.teamcode.architecture.hardware.photon.PhotonCore;

/** The runner's {@code photon} param (0 off, else the parallel-command limit) for benches that also run without it. */
final class PhotonSetup {
    private PhotonSetup() {}

    static void fromParams(BenchReport report) {
        int parallel = (int) BenchIO.param(report.params, "photon", 0);
        if (parallel > 0) {
            PhotonCore.setMaxParallelWrites(parallel);
            PhotonCore.enable();
        } else {
            PhotonCore.disable();
        }
        report.put("photonParallel", parallel);
    }
}
