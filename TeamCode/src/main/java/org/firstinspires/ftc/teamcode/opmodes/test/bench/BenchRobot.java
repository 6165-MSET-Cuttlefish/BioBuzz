package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField;
import org.firstinspires.ftc.teamcode.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.pedro.BettaConstants;

/**
 * The BioBuzz drive stack on a bare Control Hub: the real Drivetrain module and Pedro's Mecanum and Foresight with
 * BettaConstants, but a {@link SoftwareLocalizer} in place of the Pinpoint.
 */
public class BenchRobot extends Robot {
    static final double DRIVE_CAP = 0.2;

    public Drivetrain drivetrain;

    public BenchRobot(EnhancedOpMode opMode) {
        super(opMode);
        FieldConfig.setSymmetry(BioBuzzField.SYMMETRY);
    }

    static Follower softwareFollower(HardwareMap hardwareMap) {
        for (String motor : new String[] {"fl", "fr", "bl", "br"}) {
            BenchIO.require(hardwareMap, DcMotorEx.class, motor, BenchIO.RUNNER_HINT);
        }
        return new Follower(new SoftwareLocalizer(),
                new CappedMecanum(new Mecanum(hardwareMap, BettaConstants.drivetrainConfig()), DRIVE_CAP),
                new Foresight(BettaConstants.foresightConfig()));
    }

    @Override
    protected Follower createFollower(HardwareMap hardwareMap) {
        return softwareFollower(hardwareMap);
    }

    @Override
    protected void initializeGameModules() {
        BenchIO.require(opMode.hardwareMap, AnalogInput.class, "floodgate", BenchIO.RUNNER_HINT);
        drivetrain = new Drivetrain(opMode.hardwareMap).withFollower(follower);
    }
}
