package frc.robot.util.smart;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Create a number bound to a SmartDashboard value.
 * <p>
 * Inspired by {@link https://github.com/StuyPulse/StuyLib/blob/main/src/com/stuypulse/stuylib/network/SmartNumber.java}
 */
public class SmartNumber implements DoubleSupplier, Supplier<Double> {
    private double value;
    private String key;

    /**
     * Create a numerical SmartDashboard entry with the key and value.
     * 
     * @param key
     * @param value
     */
    public SmartNumber(String key, double value) {
        this.key = key;
        this.value = value;
        SmartDashboard.putNumber(key, value);
    }

    /**
     * Get the current cached value (may not match the SmartDashboard value).
     * Only use if performance from updating too often is a concern.
     * @return the cached double value
     */
    public double getCached() {
        return this.value;
    }

    /**
     * Get the current SmartDashboard value.
     * @return value
     */
    public Double get() {
        update();
        return this.value;
    }

    /**
     * Get the current SmartDashboard value.
     * @return value
     */
    public double getAsDouble() {
        return get();
    }

    /**
     * Update the cached value with the SmartDashboard value.
     */
    public void update() {
        this.value = SmartDashboard.getNumber(key, value);
    }

    /**
     * Update the cached and SmartDashboard value.
     * @param value
     */
    public void set(double value) {
        this.value = value;
        SmartDashboard.putNumber(key, value);
    }
}

