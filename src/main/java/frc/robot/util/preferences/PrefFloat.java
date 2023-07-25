package frc.robot.util.preferences;

import edu.wpi.first.wpilibj.Preferences;

public class PrefFloat {
    private float value;
    private String key;

    /**
     * Create a float preference with the provided key and value
     * <p>
     * Loads the preference into robot memory. 
     * If the robot memory already contains a value with the same key, 
     * it will overwrite the provided value.
     * 
     * @param key
     * @param value
     */
    public PrefFloat(String key, float value) {
        this.key = key;
        this.value = value;
        loadPreferences();
    }

    /**
     * Load preference from robot memory
     */
    public void loadPreferences() {
        Preferences.initFloat(key, value);
        value = Preferences.getFloat(key, value);
    }

    /**
     * Upload the current value of the preference in code to the robot memory
     */
    public void uploadPreferences() {
        Preferences.setFloat(key, value);
    }

    /**
     * Get the current value of this preference.
     * @return value
     */
    public float get() {
        return this.value;
    }

    /**
     * Set the current value of this preference.
     * @param value
     */
    public void set(float value) {
        this.value = value;
        uploadPreferences();
    }

}

