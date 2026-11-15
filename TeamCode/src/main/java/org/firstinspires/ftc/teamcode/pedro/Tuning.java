package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.OctoQuadTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

// No Foresight Tuner until the procedures are re-copied: the vendored one drives at full power with no STOP check or runaway stop.
public class Tuning {
    static final String TESTS = "AutoTune's Tests";

    @Tuner(name = "Mecanum Tuner")
    public static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner(name = "OctoQuad Tuner")
    public static Procedure octoQuad() {
        return new OctoQuadTuner();
    }

    // Every test that writes a drive motor builds the drivetrain (Odometry drives by itself), so only Pose Test runs until measured.
    @Tuner(name = "Tests")
    public static Procedure tests() {
        return new Tests(Tuning::gatedDrivetrain, BioBuzzConstants::readyLocalizer, Tuning::gatedForesight);
    }

    static Mecanum gatedDrivetrain(HardwareMap hardwareMap) {
        BioBuzzConstants.requireMeasured(TESTS);
        return BioBuzzConstants.drivetrain(hardwareMap);
    }

    static Foresight gatedForesight() {
        BioBuzzConstants.requireMeasured(TESTS);
        return BioBuzzConstants.foresight();
    }
}
