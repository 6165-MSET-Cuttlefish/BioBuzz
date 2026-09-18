package org.firstinspires.ftc.teamcode.architecture.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/** Servo wrapper with position caching. No voltage comp — PWM, not power-scaled. */
public class EnhancedServo implements Servo, PwmControl {
    private final ServoImplEx servo;
    private final WriteCache cache = new WriteCache();

    public EnhancedServo(ServoImplEx servo) {
        this.servo = servo;
        // Servo position range is 0..1, not the WriteCache default of -1..1.
        cache.min = 0.0;
        cache.max = 1.0;
    }

    public EnhancedServo(HardwareMap hardwareMap, String name) {
        this(hardwareMap.get(ServoImplEx.class, name));
    }

    public EnhancedServo withCachingTolerance(double tolerance) {
        setCachingTolerance(tolerance);
        return this;
    }

    public EnhancedServo withPositionBounds(double min, double max) {
        cache.min = min;
        cache.max = max;
        return this;
    }

    @Override
    public void setPosition(double position) {
        double corrected = cache.clamp(position);
        if (cache.shouldWrite(corrected)) {
            cache.store(corrected);
            servo.setPosition(corrected);
        }
    }

    /** Bypasses the write cache. Test-only. */
    public void setPositionRaw(double position) {
        double corrected = cache.clamp(position);
        cache.store(corrected);
        servo.setPosition(corrected);
    }

    public void setCachingTolerance(double tolerance) {
        cache.tolerance = Math.max(0.0, Math.min(1.0, tolerance));
    }

    public double getCachingTolerance() { return cache.tolerance; }
    /** Raw device; writing position through it bypasses the write cache and leaves it stale. */
    public ServoImplEx getUnderlying() { return servo; }
    public double getCachedPosition() { return cache.cached; }

    @Override public ServoController getController() { return servo.getController(); }
    @Override public int getPortNumber() { return servo.getPortNumber(); }

    @Override public Direction getDirection() { return servo.getDirection(); }
    @Override public void setDirection(Direction direction) {
        servo.setDirection(direction);
        // Direction remaps the commanded position; drop the cache so the next setPosition writes.
        cache.store(Double.NaN);
    }

    @Override public void scaleRange(double min, double max) {
        servo.scaleRange(min, max);
        // scaleRange remaps what the same logical position means; drop the cache so the next setPosition writes.
        cache.store(Double.NaN);
    }

    @Override public Manufacturer getManufacturer() { return servo.getManufacturer(); }
    @Override public String getDeviceName() { return servo.getDeviceName(); }
    @Override public String getConnectionInfo() { return servo.getConnectionInfo(); }
    @Override public int getVersion() { return servo.getVersion(); }
    @Override public void resetDeviceConfigurationForOpMode() { servo.resetDeviceConfigurationForOpMode(); }
    @Override public void close() { servo.close(); }

    @Override public double getPosition() { return servo.getPosition(); }

    @Override public void setPwmRange(PwmRange range) { servo.setPwmRange(range); }
    @Override public PwmRange getPwmRange() { return servo.getPwmRange(); }
    @Override public void setPwmEnable() { servo.setPwmEnable(); }
    @Override public void setPwmDisable() {
        servo.setPwmDisable();
        // A position write re-enables PWM; drop the cache or an identical setPosition() is suppressed and never wakes the servo.
        cache.store(Double.NaN);
    }
    @Override public boolean isPwmEnabled() { return servo.isPwmEnabled(); }

    @Override public void setPulseWidth(double microseconds) {
        servo.setPulseWidth(microseconds);
        // Raw PWM write moves the servo without going through setPosition; drop the cache so the next identical setPosition is not suppressed.
        cache.store(Double.NaN);
    }
    @Override public double getPulseWidth() { return servo.getPulseWidth(); }
}
