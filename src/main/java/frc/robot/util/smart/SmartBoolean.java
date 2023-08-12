package frc.robot.util.smart;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Create a number bound to a SmartDashboard value.
 * <p>
 * Inspired by {@link https://github.com/StuyPulse/StuyLib/blob/main/src/com/stuypulse/stuylib/network/SmartNumber.java}
 */
public class SmartBoolean implements Supplier<Boolean>, BooleanSupplier {
    private boolean value;
    private String key;

    /**
     * Create a numerical SmartDashboard entry with the key and value.
     * 
     * @param key
     * @param value
     */
    public SmartBoolean(String key, boolean value) {
        this.key = key;
        this.value = value;
        SmartDashboard.putBoolean(key, value);
    }

    /**
     * Get the current cached value (may not match the SmartDashboard value).
     * Only use if performance from updating too often is a concern.
     * @return the cached double value
     */
    public boolean getCached() {
        return this.value;
    }

    /**
     * Get the current SmartDashboard value.
     * @return value
     */
    public Boolean get() {
        update();
        return this.value;
    }

    /**
     * Get the current SmartDashboard value.
     * @return value
     */
    public boolean getAsBoolean() {
        return get();
    }

    /**
     * Update the cached value with the SmartDashboard value.
     */
    public void update() {
        this.value = SmartDashboard.getBoolean(key, value);
    }

    /**
     * Update the cached and SmartDashboard value.
     * @param value
     */
    public void set(boolean value) {
        this.value = value;
        SmartDashboard.putBoolean(key, value);
    }
}

