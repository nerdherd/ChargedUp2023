package frc.robot.util.preferences;

import edu.wpi.first.wpilibj.Preferences;

public class PrefString {
    private String value;
    private String key;

    /**
     * Create a string preference with the provided key and value
     * <p>
     * Loads the preference into robot memory. 
     * If the robot memory already contains a value with the same key, 
     * it will overwrite the provided value.
     * 
     * @param key
     * @param value
     */
    public PrefString(String key, String value) {
        this.key = key;
        this.value = value;
        loadPreferences();
    }

    /**
     * Load preference from robot memory
     */
    public void loadPreferences() {
        Preferences.initString(key, value);
        value = Preferences.getString(key, value);
    }

    /**
     * Upload the current value of the preference in code to the robot memory
     */
    public void uploadPreferences() {
        Preferences.setString(key, value);
    }

    /**
     * Get the current value of this preference.
     * @return value
     */
    public String get() {
        return this.value;
    }

    /**
     * Set the current value of this preference.
     * @param value
     */
    public void set(String value) {
        this.value = value;
        uploadPreferences();
    }

}

