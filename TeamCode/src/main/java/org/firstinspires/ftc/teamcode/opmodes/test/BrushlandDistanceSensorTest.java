package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.architecture.hardware.LaserRangefinder;

@TeleOp(name = "Brushland Distance Sensor", group = "Test")
public class BrushlandDistanceSensorTest extends OpMode {

    private LaserRangefinder lrf;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        lrf = new LaserRangefinder(hardwareMap.get(RevColorSensorV3.class, "Laser"));
        dashboard = FtcDashboard.getInstance();
        telemetry.addLine("Brushland LRF ready. Press Play.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double mm = lrf.getDistance(DistanceUnit.MM);
        double cm = lrf.getDistance(DistanceUnit.CM);
        double in = lrf.getDistance(DistanceUnit.INCH);
        int status = lrf.getStatus();

        telemetry.addData("mm",     "%.1f", mm);
        telemetry.addData("cm",     "%.2f", cm);
        telemetry.addData("in",     "%.2f", in);
        telemetry.addData("status", status);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("distance_mm", mm);
        packet.put("distance_cm", cm);
        packet.put("distance_in", in);
        packet.put("status",      status);
        dashboard.sendTelemetryPacket(packet);
    }
}
