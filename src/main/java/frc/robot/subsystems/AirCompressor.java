package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class AirCompressor extends SubsystemBase implements Reportable {
    private Compressor compressor;

    // Pressure sensor is connected to analog port on RoboRIO
    private AnalogInput pressureSensor = new AnalogInput(PneumaticsConstants.kPressureSensorPort);

    public AirCompressor() {
        compressor = new Compressor(PneumaticsConstants.kPCMPort, PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();
        // compressor.disable();
    }

    @Override
    public void periodic() {}

    public void reportToSmartDashboard() {
        SmartDashboard.putNumber("Air Pressure", pressureSensor.getValue());
    }
}
