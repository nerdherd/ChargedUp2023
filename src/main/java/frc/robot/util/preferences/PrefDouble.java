package frc.robot.util.preferences;

import edu.wpi.first.wpilibj.Preferences;

public class PrefDouble {
    private double value;
    private String key;

    /**
     * Creates a double preference with the provided key and value
     * <p>
     * Loads the preference into robot memory. 
     * If the robot memory already contains a value with the same key, 
     * it will overwrite the provided value.
     * 
     * @param key
     * @param value
     */
    public PrefDouble(String key, double value) {
        this.key = key;
        this.value = value;
        loadPreferences();
    }

    /**
     * Load preference from robot memory
     */
    public void loadPreferences() {
        Preferences.initDouble(key, value);
        value = Preferences.getDouble(key, value);
    }

    /**
     * Upload the current value of the preference in code to the robot memory
     */
    public void uploadPreferences() {
        Preferences.setDouble(key, value);
    }

    /**
     * Get the current value of this preference.
     * @return value
     */
    public double get() {
        return this.value;
    }

    /**
     * Set the current value of this preference.
     * @param value
     */
    public void set(double value) {
        this.value = value;
        uploadPreferences();
    }
}

