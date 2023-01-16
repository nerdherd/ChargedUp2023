package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode; 
import org.photonvision.PhotonUtils;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
 
public class Drivetrain extends SubsystemBase{
    private WPI_TalonFX rightMaster;
    private WPI_TalonFX leftMaster;
    private WPI_TalonFX rightFollower;
    private WPI_TalonFX leftFollower;
 
    private DifferentialDrive drive;
    private MotorControllerGroup leftMotors;
    private MotorControllerGroup rightMotors;
    
    // Vision variables
    private Vision vision;
    private PIDController turnController = new PIDController(DriveConstants.kAngularP, 0, DriveConstants.kAngularD);
    private PIDController forwardController = new PIDController(DriveConstants.kLinearP, 0, DriveConstants.kLinearD);
    private AHRS ahrs;//; = new AHRS();

    public Drivetrain(Vision vision) {
        ahrs = RobotContainer.ahrs.ahrs;

        rightMaster = new WPI_TalonFX(DriveConstants.kRightMasterID);
        leftMaster = new WPI_TalonFX(DriveConstants.kLeftMasterID);
        rightFollower = new WPI_TalonFX(DriveConstants.kRightFollowerID);
        leftFollower = new WPI_TalonFX(DriveConstants.kLeftFollowerID);

        rightMaster.setInverted(false);
        rightFollower.setInverted(false);
        leftMaster.setInverted(false);
        leftFollower.setInverted(false);
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.supplyCurrLimit.enable = true;
        config.supplyCurrLimit.triggerThresholdCurrent = 20; // the peak supply current, in amps
        config.supplyCurrLimit.triggerThresholdTime = 1.5; // the time at the peak supply current before the limit triggers, in sec
        config.supplyCurrLimit.currentLimit = 15; // the current to maintain if the peak supply limit is triggered
        rightMaster.configAllSettings(config); // apply the config settings; this selects the quadrature encoder
        rightFollower.configAllSettings(config);
        leftMaster.configAllSettings(config);
        leftFollower.configAllSettings(config);

        leftFollower.follow(leftMaster);
        rightFollower.follow(rightMaster);
        
        rightMaster = new WPI_TalonFX(DriveConstants.kRightMasterID);
        leftMaster = new WPI_TalonFX(DriveConstants.kLeftMasterID);
        rightFollower = new WPI_TalonFX(DriveConstants.kRightFollowerID);
        leftFollower = new WPI_TalonFX(DriveConstants.kLeftFollowerID);

        leftMotors = new MotorControllerGroup(leftMaster, leftFollower);
        rightMotors = new MotorControllerGroup(rightMaster, rightFollower);
        
        rightMotors.setInverted(true);
        leftMotors.setInverted(false);
        
        //check inversion to make drivetrain extend differential drive
        drive = new DifferentialDrive(leftMotors, rightMotors);
        drive.setSafetyEnabled(false);
 
        this.vision = vision;
    }
 
    public void tankDrive(double leftInput, double rightInput) {
        double prevLeftOutput = leftMaster.getMotorOutputPercent();
        double prevRightOutput = rightMaster.getMotorOutputPercent();
   
        // Curve output to quadratic
        double leftOutput = Math.abs(Math.pow(leftInput, 1)) * Math.signum(leftInput);
        double rightOutput = Math.abs(Math.pow(rightInput, 1)) * Math.signum(rightInput);
   
        // Low pass filter, output = (alpha * intended value) + (1-alpha) * previous value
        leftOutput = (DriveConstants.kDriveAlpha * leftOutput)
                    + (DriveConstants.kDriveOneMinusAlpha * prevLeftOutput);
        rightOutput = (DriveConstants.kDriveAlpha * rightOutput)
                    + (DriveConstants.kDriveOneMinusAlpha * prevRightOutput);
       
        //setPower(leftOutput*0.9, rightOutput*0.9);
        SmartDashboard.putNumber("Left Output", leftOutput);
        SmartDashboard.putNumber("Right Output", rightOutput);
        drive.tankDrive(leftInput, rightInput);
 
    }

    public void setNeutralCoast() {
        rightMaster.setNeutralMode(NeutralMode.Coast);
        leftMaster.setNeutralMode(NeutralMode.Coast);
        rightFollower.setNeutralMode(NeutralMode.Coast);
        leftFollower.setNeutralMode(NeutralMode.Coast);
    }

    public void setPower(double leftPower, double rightPower) {
        leftMaster.set(ControlMode.PercentOutput, leftPower);
        rightMaster.set(ControlMode.PercentOutput, rightPower);
        SmartDashboard.putNumber("Left Master Current", leftMaster.getStatorCurrent());
        SmartDashboard.putNumber("Left Follower Current", leftFollower.getStatorCurrent());
        SmartDashboard.putNumber("Right Master Current", rightMaster.getStatorCurrent());
        SmartDashboard.putNumber("Right Follower Current", rightFollower.getStatorCurrent());
        SmartDashboard.putNumber("Right Master Current Input", rightMaster.getSupplyCurrent());
        SmartDashboard.putNumber("Right Follower Current Input", rightFollower.getSupplyCurrent());
        // leftMotors.setVoltage(leftPower);
        // rightMotors.setVoltage(rightPower);
    }


    private double meterToTicks(double meterDist) {
        double feetDist = meterDist * 3.2808399;
        double ticks = DriveConstants.kTicksPerFoot * feetDist;
        return ticks;
    }

    public double getApriltagRotation() {
        double rotationSpeed;
        if(vision.limelightHasTargets){
            rotationSpeed = -turnController.calculate(vision.getYaw(), 0);
        }else{
        rotationSpeed = 0;
        }
        SmartDashboard.putNumber("RotationSpeed", rotationSpeed);
        return rotationSpeed;
    }
   
    public void arcadeDrive(double forwardSpeed, double rotationSpeed){
        drive.arcadeDrive(forwardSpeed, rotationSpeed);
    }

    public double getApriltagLinear(){
        double forwardSpeed;
        if(vision.limelightHasTargets){
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.kCameraHeightMeters, 
                VisionConstants.kTargetHeightMeters, 
                VisionConstants.kCameraPitchRadians, 
                Units.degreesToRadians(vision.getPitch()));

            SmartDashboard.putNumber("Range", range);
            forwardSpeed = -forwardController.calculate(range, VisionConstants.kGoalRangeMeters);
        }
        else{
            forwardSpeed = 0;
        }
        SmartDashboard.putNumber("ForwardSpeed", forwardSpeed);
        return forwardSpeed;
    }

    public double getAprilTagAreaLinear(){
        double forwardSpeed;
        if(vision.limelightHasTargets){
            double range = vision.getArea()*VisionConstants.kAreaConstant;
            forwardSpeed = - forwardController.calculate(range, VisionConstants.kGoalRangeMeters);
            SmartDashboard.putNumber("Range", range);
        }
        else{
            forwardSpeed = 0;
        }
        SmartDashboard.putNumber("ForwardSpeed", forwardSpeed);
        return forwardSpeed;
    }

    public double getHeading() {
        return ahrs.getYaw();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Rotation3d getRotation3d() {
        return new Rotation3d(
            ahrs.getRoll() * Math.PI / 180, 
            ahrs.getPitch()* Math.PI / 180, 
            ahrs.getYaw() * Math.PI / 180) ;
    }

    public void zeroHeading() {
        ahrs.reset();
    }

}
