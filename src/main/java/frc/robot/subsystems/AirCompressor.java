package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class AirCompressor extends SubsystemBase implements Reportable {
    private boolean enabled;
    private Compressor compressor;

    // Pressure sensor is connected to analog port on RoboRIO
    private AnalogInput pressureSensor = new AnalogInput(PneumaticsConstants.kPressureSensorPort);

    public AirCompressor() {
        compressor = new Compressor(PneumaticsConstants.kPCMPort, PneumaticsModuleType.CTREPCM);
        enable();
        // compressor.disable();
    }

    public boolean getEnabled() {
        return enabled;
    }

    public void setEnabled(boolean enable) {
        if (this.enabled == enable) return;

        if (enable) {
            enable();
        } else {
            disable();
        }
    }

    public void toggleEnabled() {
        setEnabled(!enabled);
    }

    public void enable() {
        this.enabled = true;
        compressor.enableDigital();
    }

    public void disable() {
        this.enabled = false;
        compressor.disable();
    }

    @Override
    public void periodic() {}

    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF || level == LOG_LEVEL.MINIMAL)  {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab("Pneumatics");

        switch (level) {
            case OFF:
                break;
            case ALL:
            case MEDIUM:
                tab.addNumber("Air Pressure", pressureSensor::getValue);
                tab.addBoolean("Compressor Enabled", () -> enabled);
                tab.add("Toggle Compressor", Commands.runOnce(this::toggleEnabled));
                tab.add("Enable Compressor", Commands.runOnce(this::enable));
                tab.add("Disable Compressor", Commands.runOnce(this::disable));
            case MINIMAL:
                break;
        }
    }

    public void reportToSmartDashboard(LOG_LEVEL level) {
        switch (level) {
            case OFF:
                break;
            case ALL:
            case MEDIUM:
                SmartDashboard.putNumber("Air Pressure", pressureSensor.getValue());
                SmartDashboard.putBoolean("Compressor Enabled", enabled);
            case MINIMAL:
                break;
        }

    }
}
