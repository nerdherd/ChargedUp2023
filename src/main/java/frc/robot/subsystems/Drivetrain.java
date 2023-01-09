package frc.robot.subsystems;
 
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants.DriveConstants;
 
public class Drivetrain{
   
    // private TalonFX rightMaster;
    // private TalonFX leftMaster;
    // private TalonFX rightFollower;
    // private TalonFX rightFollower2;
    // private TalonFX leftFollower;
    // private TalonFX leftFollower2;

    private WPI_TalonFX rightMaster;
    private WPI_TalonFX leftMaster;
    private WPI_TalonFX rightFollower;
    private WPI_TalonFX leftFollower;
 
    private DifferentialDrive drive;
    private MotorControllerGroup leftMotors;
    private MotorControllerGroup rightMotors;
 
    public Drivetrain() {

        
        rightMaster = new WPI_TalonFX(DriveConstants.kRightMasterID);
        leftMaster = new WPI_TalonFX(DriveConstants.kLeftMasterID);
        rightFollower = new WPI_TalonFX(DriveConstants.kRightFollowerID);
        leftFollower = new WPI_TalonFX(DriveConstants.kLeftFollowerID);

        leftMotors = new MotorControllerGroup(leftMaster, leftFollower);
        rightMotors = new MotorControllerGroup(rightMaster, rightFollower);
        
        rightMotors.setInverted(true);
        leftMotors.setInverted(false);
        
        // check inversion to make drivetrain extend differential drive
        drive = new DifferentialDrive(leftMaster, rightMaster);
 
    }
 
    public void tankDrive(double leftInput, double rightInput) {
        double prevLeftOutput = leftMaster.getMotorOutputPercent();
        double prevRightOutput = rightMaster.getMotorOutputPercent();
   
        // Curve output to quadratic
        double leftOutput = Math.abs(Math.pow(leftInput, 3)) * Math.signum(leftInput);
        double rightOutput = Math.abs(Math.pow(rightInput, 3)) * Math.signum(rightInput);
   
        // Low pass filter, output = (alpha * intended value) + (1-alpha) * previous value
        leftOutput = (DriveConstants.kDriveAlpha * leftOutput)
                    + (DriveConstants.kDriveOneMinusAlpha * prevLeftOutput);
        rightOutput = (DriveConstants.kDriveAlpha * rightOutput)
                    + (DriveConstants.kDriveOneMinusAlpha * prevRightOutput);
       
        drive.tankDrive(leftOutput, rightOutput);
 
    }

    public void setNeutralCoast() {
        rightMaster.setNeutralMode(NeutralMode.Coast);
        leftMaster.setNeutralMode(NeutralMode.Coast);
        rightFollower.setNeutralMode(NeutralMode.Coast);
        leftFollower.setNeutralMode(NeutralMode.Coast);
    }

    public void setPower(double leftPower, double rightPower) {
        leftMotors.setVoltage(leftPower);
        rightMotors.setVoltage(rightPower);
    }


    private double meterToTicks(double meterDist) {
        double feetDist = meterDist * 3.2808399;
        double ticks = DriveConstants.kTicksPerFoot * feetDist;
        return ticks;
    }
   
}
