package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConeRunnerConstants;

public class ConeRunner extends SubsystemBase implements Reportable{
    private TalonSRX rightMaster;
    private TalonSRX leftMaster;

    public ConeRunner(){
        rightMaster = new TalonSRX(ConeRunnerConstants.kRightMasterID);
        leftMaster = new TalonSRX(ConeRunnerConstants.kLeftMasterID);
        rightMaster.setInverted(false);
        leftMaster.setInverted(false);
    }

    public void runConeRunner(){
        rightMaster.set(ControlMode.PercentOutput, 0.3);
        leftMaster.set(ControlMode.PercentOutput,0.3); 
    }

    public void stopConeRunner(){
        rightMaster.set(ControlMode.PercentOutput, 0);
        leftMaster.set(ControlMode.PercentOutput, 0);
    }

    public void reportToSmartDashboard() {
        
    }

}
