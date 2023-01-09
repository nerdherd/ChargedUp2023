package frc.robot.subsystems;
 
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
 
public class Drivetrain {
   
    // private TalonFX rightMaster;
    // private TalonFX leftMaster;
    // private TalonFX rightFollower;
    // private TalonFX rightFollower2;
    // private TalonFX leftFollower;
    // private TalonFX leftFollower2;

    private TalonSRX rightMaster;
    private TalonSRX leftMaster;
    private VictorSPX rightFollower;
    private VictorSPX rightFollower2;
    private VictorSPX leftFollower;
    private VictorSPX leftFollower2;
 
 
    public Drivetrain() {
        rightMaster = new TalonSRX(DriveConstants.kRightMasterID);
        leftMaster = new TalonSRX(DriveConstants.kLeftMasterID);
        rightFollower = new VictorSPX(DriveConstants.kRightFollowerID);
        rightFollower2 = new VictorSPX(DriveConstants.kRightFollower2ID);
        leftFollower = new VictorSPX(DriveConstants.kLeftFollowerID);
        leftFollower2 = new VictorSPX(DriveConstants.kLeftFollower2ID);

 
        rightMaster.setInverted(true);
        leftMaster.setInverted(true);
 
        rightFollower.follow(rightMaster);
        rightFollower2.follow(rightMaster);
        leftFollower.follow(leftMaster);
        leftFollower2.follow(leftMaster);
 
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
       
        rightMaster.set(ControlMode.PercentOutput, rightOutput);
        leftMaster.set(ControlMode.PercentOutput, leftOutput); 
 
    }

    public void setNeutralCoast() {
        rightMaster.setNeutralMode(NeutralMode.Coast);
        rightFollower.setNeutralMode(NeutralMode.Coast);
        rightFollower2.setNeutralMode(NeutralMode.Coast);
        leftMaster.setNeutralMode(NeutralMode.Coast);
        leftFollower.setNeutralMode(NeutralMode.Coast);
        leftFollower2.setNeutralMode(NeutralMode.Coast);
    }

    public void setPower(double power) {
        rightMaster.set(ControlMode.PercentOutput, power);
        leftMaster.set(ControlMode.PercentOutput, power);
    }

    public void forwardDistance(double meterDist) {
        double currentPos = (rightMaster.getSelectedSensorPosition() + leftMaster.getSelectedSensorPosition()) / 2;
        double targetPos = currentPos + meterToTicks(meterDist);
        while (Math.abs(targetPos - currentPos) <= DriveConstants.kErrorBound) {
            setPower(DriveConstants.kAutoPower);
        }
    }

    public void backwardDistance(double meterDist) {
        double currentPos = (rightMaster.getSelectedSensorPosition() + leftMaster.getSelectedSensorPosition()) / 2;
        double targetPos = currentPos - meterToTicks(meterDist);
        while (Math.abs(targetPos - currentPos) <= DriveConstants.kErrorBound) {
            setPower(DriveConstants.kAutoPower);
        }
    }

    public double meterToTicks(double meterDist) {
        double feetDist = meterDist * 3.2808399;
        double ticks = DriveConstants.kTicksPerFoot * feetDist;
        return ticks;
    }
   
}
