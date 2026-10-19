package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;

class LaserRangefinder {
    public final LynxI2cDeviceSynch i2c;

    public LaserRangefinder(RevColorSensorV3 device) {
        this.i2c = (LynxI2cDeviceSynch) device.getDeviceClient();
        this.i2c.enableWriteCoalescing(true);
    }

    public void setPin0Digital(int thresh_lo, int thresh_hi) {
        setPin(R_PIN0_MODE, M_DIG, thresh_lo, thresh_hi);
    }

    public void setPin1Digital(int thresh_lo, int thresh_hi) {
        setPin(R_PIN1_MODE, M_DIG, thresh_lo, thresh_hi);
    }

    public void setPin0Analog(int bound_lo, int bound_hi) {
        setPin(R_PIN0_MODE, M_ANA, bound_lo, bound_hi);
    }

    public void setPin1Analog(int bound_lo, int bound_hi) {
        setPin(R_PIN1_MODE, M_ANA, bound_lo, bound_hi);
    }

    private void setPin(byte reg, byte reg2, int lo, int hi) {
        byte lo0 = (byte) (lo & 0xFF);
        byte lo1 = (byte) ((lo & 0xFF00) >> 8);
        byte hi0 = (byte) (hi & 0xFF);
        byte hi1 = (byte) ((hi & 0xFF00) >> 8);
        write(reg, new byte[]{reg2, lo0, lo1, hi0, hi1});
    }

    public int getPin0Mode() {
        return i2c.read8(R_PIN0_MODE);
    }

    public int getPin1Mode() {
        return i2c.read8(R_PIN1_MODE);
    }

    public void setDistanceMode(DistanceMode mode) {
        write(R_DISTMODE, new byte[]{(byte) (mode.ordinal() + 1)});
    }

    public int getDistanceModeRaw() {
        return i2c.read8(R_DISTMODE);
    }

    // A live sensor's timing budget is at least 5 ms; a silent or unplugged one reads all zeros.
    public boolean isResponding() {
        return getTiming()[0] >= 5;
    }

    public DistanceMode getDistanceMode() {
        byte v = i2c.read8(R_DISTMODE);
        switch (v) {
            case 1: return DistanceMode.SHORT;
            case 2: return DistanceMode.MEDIUM;
            case 3: return DistanceMode.LONG;
            default: throw new RuntimeException("Could not get distance mode, " + v);
        }
    }

    public void setTiming(int budget, int period) {
        if (budget < 5 || budget > 1000) throw new RuntimeException("Invalid timing budget: " + budget);
        if (period != 0 && (period < budget + 3)) throw new RuntimeException("Measurement period must be at least 4ms more than timing budget if set. For fast ranging, use a period of 0.");
        write(R_TIMING, new byte[]{(byte) (budget & 0xFF), (byte) ((budget & 0xFF00) >> 8), (byte) (period & 0xFF), (byte) ((period & 0xFF00) >> 8)});
    }

    public int[] getTiming() {
        java.nio.ByteBuffer buf = java.nio.ByteBuffer.wrap(i2c.read(R_TIMING, 4)).order(java.nio.ByteOrder.LITTLE_ENDIAN);
        return new int[]{buf.getShort(), buf.getShort()};
    }

    public void setROI(int topLeftX, int topLeftY, int botRightX, int botRightY) {
        if (botRightX - topLeftX < 3 || topLeftY - botRightY < 3) throw new RuntimeException("ROI too small, must be at least 4x4.");
        for (int v : new int[]{topLeftX, topLeftY, botRightX, botRightY}) if (v < 0 || v > 15) throw new RuntimeException("Invalid ROI point value: " + v);
        write(R_ROI_TLX, new byte[]{(byte) topLeftX, (byte) topLeftY, (byte) botRightX, (byte) botRightY});
    }

    public int[] getROI() {
        byte[] data = i2c.read(R_ROI_TLX, 4);
        return new int[]{data[0], data[1], data[2], data[3]};
    }

    public void setI2CAddress(int newAddress) {
        if (newAddress < 1 || newAddress > 127) throw new RuntimeException("Invalid I2C address: " + newAddress);
        write(R_IIC_ADDR, new byte[]{(byte) newAddress});
    }

    public int[] getOpticalCenter() {
        byte[] data = i2c.read(R_OPTCENTERX, 2);
        return new int[]{data[0], data[1]};
    }

    // Also clears a custom I2C address.
    public void setI2C() {
        write(R_PIN0_MODE, new byte[]{M_I2C, 0, 0, 0, 0});
    }

    private void write(int creg, byte[] bytes) {
        i2c.write(creg, bytes);
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public enum DistanceMode {
        SHORT,
        MEDIUM,
        LONG
    }

    private static final byte M_I2C = 0;
    private static final byte M_ANA = 1;
    private static final byte M_DIG = 2;

    private static final byte R_PIN0_MODE = 0x28;
    private static final byte R_PIN1_MODE = 0x2D;
    private static final byte R_ROI_TLX = 0x32;
    private static final byte R_DISTMODE = 0x36;
    private static final byte R_TIMING = 0x37;
    private static final byte R_IIC_ADDR = 0x3B;
    private static final byte R_OPTCENTERX = 0x3C;
}
