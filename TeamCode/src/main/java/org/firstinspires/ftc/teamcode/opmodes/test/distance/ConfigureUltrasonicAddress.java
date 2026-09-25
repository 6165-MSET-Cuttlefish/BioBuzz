package org.firstinspires.ftc.teamcode.opmodes.test.distance;

import com.qualcomm.hardware.maxbotix.MaxSonarI2CXL;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Writes a new I2C address into a MaxBotix MB1242's EEPROM, since every one ships on 0xE0. Plug in only
 * the sensor being renumbered, on "sonar": anything else on 0xE0 is renumbered too. The hardwareMap
 * always hands back 0xE0, so OpModes must {@code setI2cAddress} the new value. Pin 1 held low at
 * power-up forces 0xE0 for that power cycle, the recovery path for a lost address.
 */
@Autonomous(name = "Configure Ultrasonic Address", group = "Test")
public class ConfigureUltrasonicAddress extends LinearOpMode {

    private static final int CURRENT_ADDR_8BIT = 0xE0;

    /** Even (odd rounds down) and not 0x00, 0x50, 0xA4 or 0xAA, which the sensor rejects. */
    private static final int NEW_ADDR_8BIT = 0xE2;

    @Override
    public void runOpMode() throws InterruptedException {
        MaxSonarI2CXL sonar = hardwareMap.get(MaxSonarI2CXL.class, "sonar");
        sonar.setI2cAddress(I2cAddr.create8bit(CURRENT_ADDR_8BIT));

        telemetry.addLine("=== MB1242 address change ===");
        telemetry.addData("Current (8-bit)", "0x%02X", CURRENT_ADDR_8BIT);
        telemetry.addData("New (8-bit)",     "0x%02X", NEW_ADDR_8BIT);
        telemetry.addData("Reading now, cm", "%.0f", sonar.getDistanceSync(100, DistanceUnit.CM));
        telemetry.addLine("Press Play to write. Only one sensor should be plugged in.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        sonar.writeI2cAddrToSensorEEPROM((byte) NEW_ADDR_8BIT);
        sonar.setI2cAddress(I2cAddr.create8bit(NEW_ADDR_8BIT));
        sleep(100);

        telemetry.addData("Written", "0x%02X", NEW_ADDR_8BIT);
        telemetry.addData("Reading at new address, cm", "%.0f", sonar.getDistanceSync(100, DistanceUnit.CM));
        telemetry.addLine("If that reading is garbage, the write did not take — power-cycle and retry.");
        telemetry.update();
        sleep(5000);
    }
}
