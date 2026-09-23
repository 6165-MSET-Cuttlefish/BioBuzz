package org.firstinspires.ftc.teamcode.decode;

import static org.firstinspires.ftc.teamcode.decode.DecodeContext.blueTargetPose;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.redTargetPose;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.decode.modules.Endgame;
import org.firstinspires.ftc.teamcode.decode.modules.Magazine;
import org.firstinspires.ftc.teamcode.decode.modules.Shooter;
import org.firstinspires.ftc.teamcode.decode.modules.Turret;
import org.firstinspires.ftc.teamcode.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.pedro.CuttleDecodeConstants;

@Config("DecodeRobot")
public class DecodeRobot extends Robot {

    public Drivetrain drivetrain;
    public Endgame endgame;
    public Shooter shooter;
    public Magazine magazine;
    public Turret turret;
    public DecodeActions actions;

    public Pose targetPose;

    public static WriteToggles writeToggles = new WriteToggles();
    public static ShooterTelemetry shooterTelemetry = new ShooterTelemetry();
    public static TurretTelemetry turretTelemetry = new TurretTelemetry();
    public static DrivetrainTelemetry drivetrainTelemetry = new DrivetrainTelemetry();
    public static EndgameTelemetry endgameTelemetry = new EndgameTelemetry();
    public static MagazineTelemetry magazineTelemetry = new MagazineTelemetry();

    public DecodeRobot(EnhancedOpMode opMode) throws InterruptedException {
        super(opMode);
    }

    @Override
    protected Follower createFollower(HardwareMap hardwareMap) {
        return CuttleDecodeConstants.create(hardwareMap);
    }

    @Override
    protected void initializeGameModules() {
        targetPose = Context.allianceColor == AllianceColor.RED ? redTargetPose : blueTargetPose;
        HardwareMap hw = opMode.hardwareMap;
        drivetrain = new Drivetrain(hw).withFollower(follower);
        shooter = new Shooter(hw);
        turret = new Turret(hw).withFollower(follower);
        magazine = new Magazine(hw);
        endgame = new Endgame(hw).withDrivetrain(drivetrain);
        magazine.withEndgame(endgame);
        turret.withEndgame(endgame);
        actions = new DecodeActions(this);
    }

    public void updateWriteToggles() {
        boolean robotWriteEnabled = writeToggles.robotWrite;

        drivetrain.setWriteEnabled(robotWriteEnabled && writeToggles.drivetrainWrite);
        drivetrain.setTelemetryEnabled(drivetrainTelemetry.TOGGLE);

        endgame.setWriteEnabled(robotWriteEnabled && writeToggles.endgameWrite);
        endgame.setTelemetryEnabled(endgameTelemetry.TOGGLE);

        shooter.setWriteEnabled(robotWriteEnabled && writeToggles.shooterWrite);
        shooter.setTelemetryEnabled(shooterTelemetry.TOGGLE);

        magazine.setWriteEnabled(robotWriteEnabled && writeToggles.magazineWrite);
        magazine.setTelemetryEnabled(magazineTelemetry.TOGGLE);

        turret.setWriteEnabled(robotWriteEnabled && writeToggles.turretWrite);
        turret.setTelemetryEnabled(turretTelemetry.TOGGLE);
    }

    public static class WriteToggles {
        public boolean shooterWrite = true;
        public boolean magazineWrite = true;
        public boolean turretWrite = true;
        public boolean drivetrainWrite = true;
        public boolean endgameWrite = true;
        public boolean robotWrite = true;
    }

    public static class ShooterTelemetry {
        public boolean TOGGLE = true;
        public boolean flywheel = true;
        public boolean lut = true;
        public boolean hood = true;
        public boolean current = false;
    }

    public static class TurretTelemetry {
        public boolean TOGGLE = true;
        public boolean position = true;
        public boolean servos = false;
    }

    public static class DrivetrainTelemetry {
        public boolean TOGGLE = false;
    }

    public static class EndgameTelemetry {
        public boolean TOGGLE = true;
        public boolean current = false;
        public boolean initial = true;
        public boolean pto = false;
    }

    public static class MagazineTelemetry {
        public boolean TOGGLE = true;
        public boolean intake = false;
        public boolean vertical = false;
        public boolean servos = false;
        public boolean current = false;
        public boolean headlights = false;
        public boolean colorSensors = true;
    }
}
