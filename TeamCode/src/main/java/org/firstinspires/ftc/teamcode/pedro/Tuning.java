package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

public class Tuning {

    @Tuner(name = "Mecanum Tuner")
    public static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner(name = "Pinpoint Tuner")
    public static Procedure pinpoint() {
        return new PinpointTuner();
    }

    @Tuner(name = "Foresight Tuner")
    public static Procedure foresight() {
        return new ForesightTuner(
                hw -> new PinpointLocalizer(hw, BettaConstants.localizerConfig),
                hw -> new Mecanum(hw, BettaConstants.drivetrainConfig));
    }

    @Tuner(name = "Tests")
    public static Procedure tests() {
        return new Tests(
                hw -> new Mecanum(hw, BettaConstants.drivetrainConfig),
                hw -> new PinpointLocalizer(hw, BettaConstants.localizerConfig),
                () -> new Foresight(BettaConstants.foresightConfig));
    }
}
