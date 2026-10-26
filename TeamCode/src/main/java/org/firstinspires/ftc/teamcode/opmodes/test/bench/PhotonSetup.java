package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.seattlesolvers.solverslib.photon.PhotonCore;

/** The runner's {@code photon} param (0 off, else the parallel-command limit) for benches that also run without it. */
final class PhotonSetup {
    private PhotonSetup() {}

    static void fromParams(BenchReport report) {
        int parallel = (int) BenchIO.param(report.params, "photon", 0);
        if (parallel > 0) {
            PhotonCore.experimental.setMaximumParallelCommands(parallel);
            PhotonCore.enable();
        } else {
            PhotonCore.disable();
        }
        report.put("photonParallel", parallel);
    }
}
