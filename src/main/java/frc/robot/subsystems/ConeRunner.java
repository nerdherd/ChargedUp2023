package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConeRunnerConstants;

public class ConeRunner extends SubsystemBase implements Reportable {
    private TalonSRX positionMotor;
    private TalonSRX speedMotor;

    public ConeRunner(){
        positionMotor = new TalonSRX(ConeRunnerConstants.kPositionID);
        speedMotor = new TalonSRX(ConeRunnerConstants.kSpeedID);
        positionMotor.setInverted(false);
        speedMotor.setInverted(false);
        positionMotor.setNeutralMode(NeutralMode.Brake);
        speedMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void joystickSpeedControl(double power) {
        speedMotor.set(ControlMode.PercentOutput, power);
    }

    public void joystickAngleControl(double power) {
        positionMotor.set(ControlMode.PercentOutput, power);
    }

    public void runConeRunner(){
        speedMotor.set(ControlMode.PercentOutput,0.3); 
    }

    public void stopConeRunner(){
        speedMotor.set(ControlMode.PercentOutput, 0);
    }

    public void reportToSmartDashboard() {
        
    }

}
