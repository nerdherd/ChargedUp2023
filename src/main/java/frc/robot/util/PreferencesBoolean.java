package frc.robot.util;

import edu.wpi.first.wpilibj.Preferences;

public class PreferencesBoolean {
    
    private boolean value;
    private String key;

    public PreferencesBoolean(String key, boolean value) {
        this.key = key;
        this.value = value;
        loadPreferences();
    }

    public void loadPreferences() {

        if ((Preferences.containsKey(key) == true) && (value != Preferences.getBoolean(key, value))) {
            value = Preferences.getBoolean(key, value);
        } else {
            Preferences.initBoolean(key, value);
            value = Preferences.getBoolean(key, value);
        }

    }

    public void uploadPreferences() {
        Preferences.setBoolean(key, value);
    }

    public boolean get() {
        return Preferences.getBoolean(key, value);
    }

    public void set() {
        uploadPreferences();
    }

}

