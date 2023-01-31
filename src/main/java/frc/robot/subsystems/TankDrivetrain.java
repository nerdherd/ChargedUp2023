package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import java.util.HashMap;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
 
public class TankDrivetrain extends SubsystemBase{
    private TalonFX rightMaster;
    private TalonFX leftMaster;
    private TalonFX rightFollower;
    private TalonFX leftFollower;
 
    private DifferentialDrive drive;
    private MotorControllerGroup leftMotors;
    private MotorControllerGroup rightMotors;
    
    // Vision variables
    private Vision vision;
    private PIDController turnController = new PIDController(DriveConstants.kAngularP, 0, DriveConstants.kAngularD);
    private PIDController forwardController = new PIDController(DriveConstants.kLinearP, 0, DriveConstants.kLinearD);
    private AHRS ahrs;//; = new AHRS();
    private DoubleSolenoid shifter;

    public TankDrivetrain(Vision vision) {
        ahrs = RobotContainer.imu.ahrs;
        
        shifter = new DoubleSolenoid(ClawConstants.kPCMPort, PneumaticsModuleType.CTREPCM, 
            DriveConstants.kPistonForwardID, DriveConstants.kPistonReverseID);

        rightMaster = new TalonFX(DriveConstants.kLeftFollowerID);
        leftMaster = new TalonFX(DriveConstants.kRightFollowerID);
        rightFollower = new TalonFX(DriveConstants.kLeftFollower2ID);
        leftFollower = new TalonFX(DriveConstants.kRightFollower2ID);

        rightMaster.setInverted(false);
        rightFollower.setInverted(false);
        leftMaster.setInverted(true);
        leftFollower.setInverted(true);
        
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
        
        // rightMaster = new WPI_TalonFX(DriveConstants.kRightMasterID);
        // leftMaster = new WPI_TalonFX(DriveConstants.kLeftMasterID);
        // rightFollower = new WPI_TalonFX(DriveConstants.kRightFollowerID);
        // leftFollower = new WPI_TalonFX(DriveConstants.kLeftFollowerID);

        // leftMotors = new MotorControllerGroup(leftMaster, leftFollower);
        // rightMotors = new MotorControllerGroup(rightMaster, rightFollower);
        
        // rightMotors.setInverted(true);
        // leftMotors.setInverted(false);
        
        // check inversion to make drivetrain extend differential drive
        // drive = new DifferentialDrive(leftMaster, rightMaster);
 
        this.vision = vision;
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
       

        //setPower(leftOutput*0.9, rightOutput*0.9);
        SmartDashboard.putNumber("Left Output", leftOutput);
        SmartDashboard.putNumber("Right Output", rightOutput);
        // setPower(leftInput, rightInput);
        // if (leftInput > 0.4 && rightInput < 0.4) {
        //     setPower(0.5, 0.5);
        // } else if (rightInput > 0.4) {
        //     setPower(0, 0);
        // }
        if (leftOutput >= 0.6) {
            leftOutput = 0.6;
        }
        if (rightOutput >= 0.6) {
            rightOutput = 0.6;
        }
        setPower(leftOutput, rightOutput);
    }

    boolean shiftedHigh;
    public CommandBase shiftHigh() {
        return runOnce(
            () -> {
                shifter.set(Value.kForward);
                shiftedHigh = true;
            });
    }

    public CommandBase shiftLow() {
        return runOnce(
            () -> {
                shifter.set(Value.kReverse);
                shiftedHigh = false;
            });
    }

    public void setNeutralCoast() {
        rightMaster.setNeutralMode(NeutralMode.Coast);
        leftMaster.setNeutralMode(NeutralMode.Coast);
        rightFollower.setNeutralMode(NeutralMode.Coast);
        leftFollower.setNeutralMode(NeutralMode.Coast);
    }

    public void setPower(double power) {
        leftMaster.set(ControlMode.PercentOutput, power);
        rightMaster.set(ControlMode.PercentOutput, power);
        SmartDashboard.putNumber("Left Power", power);
        SmartDashboard.putNumber("Right Power", power);

        SmartDashboard.putNumber("Left Master Current", leftMaster.getStatorCurrent());
        SmartDashboard.putNumber("Left Follower Current", leftFollower.getStatorCurrent());
        SmartDashboard.putNumber("Right Master Current", rightMaster.getStatorCurrent());
        SmartDashboard.putNumber("Right Follower Current", rightFollower.getStatorCurrent());
        SmartDashboard.putNumber("Right Master Current Input", rightMaster.getSupplyCurrent());
        SmartDashboard.putNumber("Right Follower Current Input", rightFollower.getSupplyCurrent());
        
    }

    public void setPower(double leftPower, double rightPower) {
        leftMaster.set(ControlMode.PercentOutput, leftPower);
        rightMaster.set(ControlMode.PercentOutput, rightPower);
        SmartDashboard.putNumber("Left Power", leftPower);
        SmartDashboard.putNumber("Right Power", rightPower);

        SmartDashboard.putNumber("Left Master Current", leftMaster.getStatorCurrent());
        SmartDashboard.putNumber("Left Follower Current", leftFollower.getStatorCurrent());
        SmartDashboard.putNumber("Right Master Current", rightMaster.getStatorCurrent());
        SmartDashboard.putNumber("Right Follower Current", rightFollower.getStatorCurrent());
        SmartDashboard.putNumber("Right Master Current Input", rightMaster.getSupplyCurrent());
        SmartDashboard.putNumber("Right Follower Current Input", rightFollower.getSupplyCurrent());
        // leftMotors.setVoltage(leftPower);
        // rightMotors.setVoltage(rightPower);
    }

    public void resetEncoder() {
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
    }

    public void setEncoder(double position) {
        leftMaster.setSelectedSensorPosition(position);
        rightMaster.setSelectedSensorPosition(position);
    }

    public double getTicks() {
        return leftMaster.getSelectedSensorPosition();
    }


    public double meterToTicks(double meterDist) {
        double ticks = DriveConstants.kTicksPerMeter * meterDist;
        return ticks;
    }

    public double ticksToMeters(double ticks) {
        double meterDist = ticks / DriveConstants.kTicksPerMeter;
        return meterDist;
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

    public void initShuffleboard() {  
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        ShuffleboardLayout power =
            tab.getLayout("Power", BuiltInLayouts.kGrid)
                .withSize(2, 2)
                .withProperties(new HashMap<String, Object>() {{
                    put("Number of columns", 2);
                    put("Number of rows", 1);
                    }});
        
        HashMap<String, Object> powerProperties = new HashMap<String, Object>() {{
            put("Min" , 0);
            put("Max", 1);
        }};
            
        power.addNumber("Left Master Velocity", () -> leftMaster.getSelectedSensorVelocity())
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(powerProperties);
        
        power.addNumber("Left Follower Velocity", () -> leftFollower.getSelectedSensorVelocity())
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(powerProperties);        

        power.addNumber("Right Master Velocity", () -> rightMaster.getSelectedSensorPosition())
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(powerProperties);

        power.addNumber("Right Follower Velocity", () -> rightFollower.getSelectedSensorPosition())
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(powerProperties);
        
        // ========== CURRENT LAYOUT ========== //

        ShuffleboardLayout current = 
            tab.getLayout("Current", BuiltInLayouts.kGrid)
                .withSize(3, 3)
                .withProperties(new HashMap<String, Object>() {{
                    put("Number of columns", 2);
                    put("Number of rows", 2);
                    }});
        
        HashMap<String, Object> falconCurrent = new HashMap<String, Object>() {{
            put("Min" , 0);
            put("Max", DriveConstants.kFalconMaxCurrent);
        }};
        current.addNumber("Left Master Current", leftMaster::getStatorCurrent)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(falconCurrent);
        current.addNumber("Left Follower Current", leftFollower::getStatorCurrent)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(falconCurrent);
        current.addNumber("Right Master Current", rightMaster::getStatorCurrent)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(falconCurrent);
        current.addNumber("Right Follower Current", rightFollower::getStatorCurrent)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(falconCurrent);
    }
    
    

}
