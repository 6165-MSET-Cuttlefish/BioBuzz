package org.firstinspires.ftc.teamcode.architecture.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoController;

/** CRServo wrapper with the same power-caching + voltage-comp shape as {@link EnhancedMotor}. */
public class EnhancedCRServo implements CRServo, PwmControl {
    private final CRServoImplEx crServo;
    private final WriteCache cache = new WriteCache();

    public EnhancedCRServo(CRServoImplEx crServo) {
        this.crServo = crServo;
    }

    public EnhancedCRServo(HardwareMap hardwareMap, String name) {
        this.crServo = hardwareMap.get(CRServoImplEx.class, name);
    }

    public EnhancedCRServo withCachingTolerance(double tolerance) {
        setCachingTolerance(tolerance);
        return this;
    }

    public EnhancedCRServo withPowerBounds(double min, double max) {
        cache.min = min;
        cache.max = max;
        return this;
    }

    public EnhancedCRServo withVoltageCompensation(double referenceVoltage) {
        cache.voltageCompensationEnabled = true;
        cache.referenceVoltage = referenceVoltage;
        return this;
    }

    public static void updateVoltage(double voltage) {
        BatteryVoltage.update(voltage);
    }

    @Override
    public void setPower(double power) {
        double corrected = cache.clamp(cache.applyVoltageScaling(power));
        if (cache.shouldWrite(corrected)) {
            cache.store(corrected);
            crServo.setPower(corrected);
        }
    }

    /** Bypasses voltage compensation and the write cache. Test-only. */
    public void setPowerRaw(double power) {
        double corrected = cache.clamp(power);
        cache.store(corrected);
        crServo.setPower(corrected);
    }

    public void setCachingTolerance(double tolerance) {
        cache.tolerance = Math.max(0.0, Math.min(1.0, tolerance));
    }

    public double getCachingTolerance() { return cache.tolerance; }

    public void setReferenceVoltage(double voltage) { cache.referenceVoltage = voltage; }
    public double getReferenceVoltage() { return cache.referenceVoltage; }

    public void setVoltageCompensationEnabled(boolean enabled) {
        cache.voltageCompensationEnabled = enabled;
    }

    public boolean isVoltageCompensationEnabled() {
        return cache.voltageCompensationEnabled;
    }

    /** Raw device; writing power through it bypasses the write cache and leaves it stale. */
    public CRServoImplEx getUnderlying() { return crServo; }
    public double getCachedPower() { return cache.cached; }

    @Override public ServoController getController() { return crServo.getController(); }
    @Override public int getPortNumber() { return crServo.getPortNumber(); }

    @Override public Direction getDirection() { return crServo.getDirection(); }
    @Override public void setDirection(Direction direction) {
        crServo.setDirection(direction);
        // Direction changes what the same number means, not the number; drop the cache to force a re-issue.
        cache.store(Double.NaN);
    }

    @Override public double getPower() { return crServo.getPower(); }

    @Override public Manufacturer getManufacturer() { return crServo.getManufacturer(); }
    @Override public String getDeviceName() { return crServo.getDeviceName(); }
    @Override public String getConnectionInfo() { return crServo.getConnectionInfo(); }
    @Override public int getVersion() { return crServo.getVersion(); }
    @Override public void resetDeviceConfigurationForOpMode() { crServo.resetDeviceConfigurationForOpMode(); }
    @Override public void close() { crServo.close(); }

    @Override public void setPwmRange(PwmRange range) { crServo.setPwmRange(range); }
    @Override public PwmRange getPwmRange() { return crServo.getPwmRange(); }
    @Override public void setPwmEnable() { crServo.setPwmEnable(); }
    @Override public void setPwmDisable() {
        crServo.setPwmDisable();
        // A power write re-enables PWM; drop the cache or an identical setPower() is suppressed and never re-powers the CRServo.
        cache.store(Double.NaN);
    }
    @Override public boolean isPwmEnabled() { return crServo.isPwmEnabled(); }

    @Override public void setPulseWidth(double microseconds) {
        crServo.setPulseWidth(microseconds);
        // Raw PWM write bypasses setPower; drop the cache so the next identical setPower is not suppressed.
        cache.store(Double.NaN);
    }
    @Override public double getPulseWidth() { return crServo.getPulseWidth(); }
}
