package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.maxbotix.MaxSonarI2CXL;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * One-shot address-change opmode for the MaxBotix MB1242 (I2CXL-MaxSonar-EZ4).
 *
 * Every MB1242 ships on 8-bit address 0xE0, so two of them on the same I2C bus collide. This
 * writes a new address into one sensor's EEPROM, where it persists across power cycles.
 *
 * Usage:
 *  1. Plug in exactly ONE sensor — the one being renumbered — on the port configured as "sonar".
 *     Anything else still on 0xE0 will be renumbered too.
 *  2. Set NEW_ADDR_8BIT below, deploy, run once.
 *  3. Verify the post-write reading below is sane (a hand held ~50 cm away should read ~50).
 *  4. In the opmode that uses this sensor, call setI2cAddress with the same value after pulling
 *     it out of the hardwareMap — the config file always hands you the 0xE0 default.
 *
 * Address rules: 8-bit, even only (odd values round down to the next even). 0x00, 0x50, 0xA4,
 * and 0xAA are rejected and leave the sensor where it was. 0xE2, 0xE4, 0xE6 ... are the obvious
 * picks for a second/third/fourth sensor.
 *
 * Note the MB1242 also has a hardware escape hatch: pin 1 pulled low at power-up makes the
 * sensor use 0xE0 for that power cycle regardless of what's in EEPROM. That's the recovery path
 * if a sensor gets written to an address nothing can find.
 *
 * Datasheet: https://maxbotix.com/pages/i2cxl-maxsonar-ez-datasheet
 */
@Autonomous(name = "Configure Ultrasonic Address", group = "Test")
public class ConfigureUltrasonicAddress extends LinearOpMode {

    /** Address the sensor is on right now. 0xE0 unless it's already been renumbered. */
    private static final int CURRENT_ADDR_8BIT = 0xE0;

    /** Address to write. Must be even and not one of 0x00 / 0x50 / 0xA4 / 0xAA. */
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
