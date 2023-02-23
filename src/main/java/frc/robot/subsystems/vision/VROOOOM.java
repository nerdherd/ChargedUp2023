package frc.robot.subsystems.vision;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.util.NerdyMath;

public class VROOOOM extends SubsystemBase implements Reportable{

    private Limelight limelightHigh;
    private Limelight limelightLow;
    private Limelight currentLimelight;

    public enum OBJECT_TYPE
    {
        CONE,
        CUBE
    }
    
    public enum CAMERA_MODE
    {
        WAIT, // found nothing
        IDLE, // doing nothing, init
        ACTION,// detected one, and approach to it
        ARRIVED // found
    }

    public enum SCORE_POS
    {
        LOW,
        MID,
        HIGH
    }

    private OBJECT_TYPE currentGameObject;
    private SCORE_POS currentHeightPos;
    private CAMERA_MODE currentCameraMode;
    private double goalArea;
    private double goalTX;
    private double goalYaw;
    private boolean rotationIsNeeded;
    public BooleanSupplier cameraStatusSupplier;
    
    private Arm arm;
    private Elevator elevator;
    private Claw claw;
    private SwerveDrivetrain drivetrain;

    public VROOOOM(Arm arm, Elevator elevator, Claw claw) {
        this.arm = arm;
        this.elevator = elevator;
        this.claw = claw;

        // defaults
        goalArea = 0;
        goalTX = 0;
        goalYaw = 0;
        rotationIsNeeded = false;
        currentGameObject = OBJECT_TYPE.CONE;
        currentHeightPos = SCORE_POS.HIGH;
        currentCameraMode = CAMERA_MODE.IDLE;
        cameraStatusSupplier = () -> (currentCameraMode == CAMERA_MODE.ARRIVED);

        try {
            limelightHigh = new Limelight("limelight-high");
            limelightHigh.setLightState(Limelight.LightMode.OFF);
        } catch (Exception ex) {
            limelightHigh = null;
            DriverStation.reportWarning("Error instantiating high camera:  " + ex.getMessage(), true);
        }

        try {
            limelightLow = new Limelight("limelight-low");
            limelightLow.setLightState(Limelight.LightMode.OFF);
        } catch (Exception ex) {
            limelightLow = null;
            DriverStation.reportWarning("Error instantiating low camera:  " + ex.getMessage(), true);
        }

        currentLimelight = limelightLow;
        currentCameraMode = CAMERA_MODE.IDLE;
        cameraStatusSupplier = () -> (currentCameraMode == CAMERA_MODE.ARRIVED);

    }

    private double Xarray[] = new double[10];
    private int xIndex = 0;
    private boolean initDoneX = false;

    public double getAvgTX(double newValue)
    {
        Xarray[xIndex] = newValue;
        xIndex ++;
        if(xIndex >= Xarray.length) {
            xIndex = 0;
            initDoneX = true;
        }

        double TXSum = 0;
        if(initDoneX) {
            for(int i = 0; i < Xarray.length; i++) {
                TXSum += Xarray[i];
            }

            return TXSum / Xarray.length;
        }
        else {
            for(int i = 0; i < xIndex; i++) {
                TXSum += Xarray[i];
            }

            return TXSum / xIndex;
        }
    }

    private double areaArray[] = new double[10];
    private int areaIndex = 0;
    private boolean initDoneArea = false;

    public double getAvgArea(double newValue)
    {
        areaArray[areaIndex] = newValue;
        areaIndex ++;
        if(areaIndex >= areaArray.length) {
            areaIndex = 0;
            initDoneArea = true;
        }

        double TYSum = 0;
        if(initDoneArea) {
            for(int i = 0; i < areaArray.length; i++) {
                TYSum += areaArray[i];
            }

            return TYSum / areaArray.length;
        }
        else {
            for(int i = 0; i < areaIndex; i++) {
                TYSum += areaArray[i];
            }

            return TYSum / areaIndex;
        }
    }

     
    public void initVisionCommands() {
        currentCameraMode = CAMERA_MODE.IDLE;
        initDoneX = false;
        initDoneArea = false;
        xIndex = 0;
        areaIndex = 0;
    }

    public SequentialCommandGroup VisionPickup() {
        // PLACEHOLDERS
        int armEnum;
        PIDController PIDArea;
        PIDController PIDTX;
        PIDController PIDYaw;

        rotationIsNeeded = false; // Reset rotation variable

        switch(currentHeightPos) {
            case HIGH:
                armEnum = 0; // Pickup substation
                currentLimelight = limelightHigh;
                rotationIsNeeded = true;
                goalYaw = 0; // Facing away from drivers, towards substation

                if (currentGameObject == OBJECT_TYPE.CONE) {
                    goalArea = 0; // Goal area for cone substation pickup
                } else {
                    goalArea = 0; // Goal area for cube substation pickup
                }
                break;

            case LOW:
                armEnum = 1; // Ground pickup
                currentLimelight = limelightLow;
                rotationIsNeeded = false;

                if (currentGameObject == OBJECT_TYPE.CONE) {
                    goalArea = 0; // Goal area for cone ground pickup
                } else {
                    goalArea = 0; // Goal area for cube ground pickup
                }
                break;

            default:

                break;
        }

        switch(currentGameObject) {
            case CONE:
                currentLimelight.setPipeline(1);
                PIDArea = new PIDController(0, 0, 0);
                PIDTX = new PIDController(0, 0, 0);
                PIDYaw = new PIDController(0, 0, 0);
                break;

            case CUBE:
                currentLimelight.setPipeline(2);
                PIDArea = new PIDController(0, 0, 0);
                PIDTX = new PIDController(0, 0, 0);
                PIDYaw = new PIDController(0, 0, 0);
                break;
        }
        
        return new SequentialCommandGroup(
            // GotoTarget()
            // Arm.gotoSetPos()
        );
    }

    public CommandBase VisionScore() {
        // PLACEHOLDERS
        int armEnum;
        PIDController PIDArea;
        PIDController PIDTX;
        PIDController PIDYaw;

        rotationIsNeeded = true;
        goalYaw = 180; // All scoring is facing towards our drivers

        switch(currentGameObject) {
            case CONE:
                currentLimelight = limelightHigh;
                currentLimelight.setPipeline(3);
                PIDArea = new PIDController(0, 0, 0);
                PIDTX = new PIDController(0, 0, 0);
                PIDYaw = new PIDController(0, 0, 0);
                break;

            case CUBE:
                currentLimelight = limelightLow;
                currentLimelight.setPipeline(4);
                PIDArea = new PIDController(0, 0, 0);
                PIDTX = new PIDController(0, 0, 0);
                PIDYaw = new PIDController(0, 0, 0);
                break;
        }

        switch(currentHeightPos) {
            case HIGH:
                armEnum = 2; // Score high
                break;

            case MID:
                armEnum = 3; // Score mid
                break;

            case LOW: // Score ground
                armEnum = 4;
                break;
        }

        return new SequentialCommandGroup(
                
        );
    }

    public void driveRotateToTarget(PIDController pidArea, PIDController pidTX, PIDController pidYaw) {
        // Initialize all variables to 0
        double xSpeed = 0;
        double ySpeed = 0;
        double rotationSpeed = 0;

        if (currentLimelight == null)
            return;

        ChassisSpeeds chassisSpeeds;

        SmartDashboard.putBoolean("Vision has target", currentLimelight.hasValidTarget());

        if(!currentLimelight.hasValidTarget()) {
            chassisSpeeds = new ChassisSpeeds(0, 0, 0);
            SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrain.setModuleStates(moduleStates);
            currentCameraMode = CAMERA_MODE.WAIT;
        }
        else {
            double calculatedX = getAvgArea(currentLimelight.getArea_avg());
            double calculatedY = getAvgTX(currentLimelight.getXAngle_avg());
            SmartDashboard.putNumber("Vision average X", calculatedX);
            SmartDashboard.putNumber("Vision average Y", calculatedY);

            xSpeed = pidArea.calculate(calculatedX, goalArea);
            ySpeed = -pidTX.calculate(calculatedY, goalTX);
            rotationSpeed = pidYaw.calculate(drivetrain.getImu().getHeading(), goalYaw);
            
            if (NerdyMath.inRange(xSpeed, -.1, .1) &&
            NerdyMath.inRange(ySpeed, -.1, .1) &&
            NerdyMath.inRange(rotationSpeed, -.1, .1))
            {
                chassisSpeeds = new ChassisSpeeds(0, 0, 0);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                currentCameraMode = CAMERA_MODE.ARRIVED; 
            }
            else{
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                currentCameraMode = CAMERA_MODE.ACTION;
            }
        }
        
        SmartDashboard.putString("Vision status", currentCameraMode.toString());
        SmartDashboard.putNumber("Vision X speed", xSpeed);
        SmartDashboard.putNumber("Vision Y speed", ySpeed);
    }

    // Drive to target without rotation
    public void skrttttToTarget(PIDController pidArea, PIDController pidTX) {
        // Initialize all variables to 0
        double xSpeed = 0;
        double ySpeed = 0;

        if (currentLimelight == null)
            return;

        ChassisSpeeds chassisSpeeds;

        SmartDashboard.putBoolean("Vision has target", currentLimelight.hasValidTarget());

        if(!currentLimelight.hasValidTarget()) {
            chassisSpeeds = new ChassisSpeeds(0, 0, 0);
            SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrain.setModuleStates(moduleStates);
            currentCameraMode = CAMERA_MODE.WAIT;
        }
        else {
            double calculatedX = getAvgArea(currentLimelight.getArea_avg());
            double calculatedY = getAvgTX(currentLimelight.getXAngle_avg());
            SmartDashboard.putNumber("Vision average X", calculatedX);
            SmartDashboard.putNumber("Vision average Y", calculatedY);

            xSpeed = pidArea.calculate(calculatedX, goalArea);
            ySpeed = -pidTX.calculate(calculatedY, goalTX);
            
            if (NerdyMath.inRange(xSpeed, -.1, .1) &&
            NerdyMath.inRange(ySpeed, -.1, .1))
            {
                chassisSpeeds = new ChassisSpeeds(0, 0, 0);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                currentCameraMode = CAMERA_MODE.ARRIVED; 
            }
            else{
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                currentCameraMode = CAMERA_MODE.ACTION;
            }
        }
        
        SmartDashboard.putString("Vision status", currentCameraMode.toString());
        SmartDashboard.putNumber("Vision X speed", xSpeed);
        SmartDashboard.putNumber("Vision Y speed", ySpeed);
    }

    @Override
    public void reportToSmartDashboard() {
        SmartDashboard.putString("Vision Current Object", currentGameObject.toString());
        SmartDashboard.putString("Vision Current Limelight", currentLimelight.getName());
        SmartDashboard.putString("Vision Current Height", currentHeightPos.toString());
    }

    @Override
    public void initShuffleboard() {
    }
}
