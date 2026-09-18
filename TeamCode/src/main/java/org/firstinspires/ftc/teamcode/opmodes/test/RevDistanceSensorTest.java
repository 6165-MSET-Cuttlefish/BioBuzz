package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Rev Distance Sensor", group = "Test")
public class RevDistanceSensorTest extends OpMode {

    private DistanceSensor sensor;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        sensor = hardwareMap.get(DistanceSensor.class, "distance");
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        double mm = sensor.getDistance(DistanceUnit.MM);
        double cm = sensor.getDistance(DistanceUnit.CM);
        double in = sensor.getDistance(DistanceUnit.INCH);

        telemetry.addData("mm", "%.1f", mm);
        telemetry.addData("cm", "%.2f", cm);
        telemetry.addData("in", "%.2f", in);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("distance_mm", mm);
        packet.put("distance_cm", cm);
        packet.put("distance_in", in);
        dashboard.sendTelemetryPacket(packet);
    }
}
