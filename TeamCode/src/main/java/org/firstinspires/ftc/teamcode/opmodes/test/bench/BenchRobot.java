package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField;
import org.firstinspires.ftc.teamcode.biobuzz.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.pedro.BioBuzzConstants;

import java.util.Set;
import java.util.function.Function;

public class BenchRobot extends Robot {
    static final double DRIVE_CAP = 0.2;
    static final String[] ROBOT_ONLY_DEVICES = {"intake", "transfer"};

    public Drivetrain drivetrain;

    public BenchRobot(EnhancedOpMode opMode) {
        super(opMode);
        FieldConfig.setSymmetry(BioBuzzField.SYMMETRY);
    }

    static Follower softwareFollower(HardwareMap hardwareMap) {
        return new Follower(new SoftwareLocalizer(), cappedDrivetrain(hardwareMap),
                new Foresight(BioBuzzConstants.foresightConfig()));
    }

    static Follower drivingFollower(HardwareMap hardwareMap) {
        requireBenchConfig(hardwareMap);
        return softwareFollower(hardwareMap);
    }

    /** The only way a bench gets a follower on a real localizer: refused off the bench runner's config, capped on it. */
    static Follower benchFollower(HardwareMap hardwareMap, Function<HardwareMap, Localizer> localizer) {
        requireBenchConfig(hardwareMap);
        return new Follower(localizer.apply(hardwareMap), cappedDrivetrain(hardwareMap),
                new Foresight(BioBuzzConstants.foresightConfig()));
    }

    private static CappedMecanum cappedDrivetrain(HardwareMap hardwareMap) {
        for (String motor : new String[] {"fl", "fr", "bl", "br"}) {
            BenchIO.require(hardwareMap, DcMotorEx.class, motor, BenchIO.RUNNER_HINT);
        }
        return new CappedMecanum(new Mecanum(hardwareMap, BioBuzzConstants.drivetrainConfig()), DRIVE_CAP);
    }

    // biobuzz.xml names no benchServo, so a bench that drives the wheels with nobody at the sticks won't start on the robot.
    static void requireBenchConfig(HardwareMap hardwareMap) {
        if (!hardwareMap.getAllNames(Servo.class).contains("benchServo")) {
            throw new IllegalStateException("No Servo \"benchServo\" in the active hardware config: this bench powers the "
                    + "drive motors with nobody at the sticks, so it runs only on the bench runner's config: " + BenchIO.RUNNER_HINT);
        }
        // A config can name benchServo and still be the robot's; these motors are on the robot and in no bench config.
        Set<String> names = hardwareMap.getAllNames(HardwareDevice.class);
        for (String device : ROBOT_ONLY_DEVICES) {
            if (names.contains(device)) {
                throw new IllegalStateException("The active hardware config names the robot's \"" + device + "\" motor: "
                        + "this bench powers the drive motors with nobody at the sticks, so it runs only on the bench "
                        + "runner's config: " + BenchIO.RUNNER_HINT);
            }
        }
    }

    @Override
    protected Follower createFollower(HardwareMap hardwareMap) {
        return drivingFollower(hardwareMap);
    }

    @Override
    protected void initializeGameModules() {
        BenchIO.require(opMode.hardwareMap, AnalogInput.class, "floodgate", BenchIO.RUNNER_HINT);
        drivetrain = new Drivetrain(opMode.hardwareMap, follower);
    }
}
