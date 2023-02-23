package frc.robot.subsystems.vision;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.NerdyMath;

// Subsystem integration notes from Ayaka
// arm.movearmmotionmagic(ticks, elevator.percentextended)
// elevator.movemotionmagic(ticks, arm.getarmangle)
// claw negative is intake at 30%

public class VROOOOM extends SubsystemBase implements Reportable{

    private Limelight limelightHigh;
    private Limelight limelightLow;
    private Limelight currentLimelight;

    // Pipelines that should be set on each limelight: cone = 1, cube = 2, tape = 3, tag = 4

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
        LOW, // Can be ground in pickup mode
        MID,
        HIGH // Can be substation in pickup mode
    }

    private OBJECT_TYPE currentGameObject;
    private SCORE_POS currentHeightPos;
    private CAMERA_MODE currentCameraMode;
    private double goalArea;
    private double goalTX;
    private double goalYaw;
    private boolean rotationIsNeeded;
    public BooleanSupplier cameraStatusSupplier;

    // Current PID Controllers
    private PIDController PIDArea = new PIDController(0, 0, 0);
    private PIDController PIDTX = new PIDController(0, 0, 0);
    private PIDController PIDYaw = new PIDController(0, 0, 0);
    
    private Arm arm;
    private Elevator elevator;
    private Claw claw;
    private SwerveDrivetrain drivetrain;

    public VROOOOM(Arm arm, Elevator elevator, Claw claw, SwerveDrivetrain drivetrain) {
        this.arm = arm;
        this.elevator = elevator;
        this.claw = claw;
        this.drivetrain = drivetrain;

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
        // PLACEHOLDER
        int armEnum;

        rotationIsNeeded = false; // Reset rotation variable

        switch(currentHeightPos) {
            case HIGH:
                armEnum = 0; // Pickup substation
                currentLimelight = limelightHigh;
                rotationIsNeeded = true;
                goalYaw = 0; // Facing away from drivers, towards substation

                if (currentGameObject == OBJECT_TYPE.CONE) {
                    goalArea = 21; // Goal area for cone substation pickup, area is an estimate because a different camera position was used, updated 2/23/2023
                } else {
                    goalArea = 0; // Goal area for cube substation pickup
                }
                break;

            case LOW:
                armEnum = 1; // Ground pickup
                currentLimelight = limelightLow;
                rotationIsNeeded = false;

                if (currentGameObject == OBJECT_TYPE.CONE) {
                    goalArea = 21; // Goal area for cone ground pickup, area is an estimate because a different camera position was used, updated 2/23/2023
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
                PIDArea = new PIDController(0.05, 0.005, 0); // Correct PID as of 2/23/2023
                PIDTX = new PIDController(0.24, 0, 0);
                PIDYaw = new PIDController(0, 0, 0);
                break;

            case CUBE:
                currentLimelight.setPipeline(2);
                PIDArea = new PIDController(0.05, 0.005, 0); // PID coppied from cone (above) as of 2/23/2023
                PIDTX = new PIDController(0.24, 0, 0);
                PIDYaw = new PIDController(0, 0, 0);
                break;
        }

        // Had to declare both RunCommands in advance because syntax errors would appear if they weren't
        RunCommand driveRotateToTargetRunCommand = new RunCommand(() -> driveRotateToTarget(PIDArea, PIDTX, PIDYaw), arm, elevator, claw, drivetrain);
        RunCommand driveToTargetRunCommand = new RunCommand(() -> skrttttToTarget(PIDArea, PIDTX), arm, elevator, claw, drivetrain);
        RunCommand currentVisionRunCommand;

        if (rotationIsNeeded) {
            currentVisionRunCommand = driveRotateToTargetRunCommand;
        } else {
            currentVisionRunCommand = driveToTargetRunCommand;
        }

        InstantCommand init = new InstantCommand(() -> initVisionCommands());

        return new SequentialCommandGroup(
            new InstantCommand(() -> SmartDashboard.putBoolean("Vision Pickup Running", true)),
            init,
            // Move arm and elevator to arm enum position
            // Open claw/Start claw intake rollers
            currentVisionRunCommand.until(cameraStatusSupplier).withTimeout(30), // Timeout after 30 seconds
            // Close claw/stop claw intake rollers/low background rolling to keep control of game piece
            // Stow arm/elev
            new InstantCommand(() -> SmartDashboard.putBoolean("Vision Pickup Running", false))
        );
    }

    public SequentialCommandGroup VisionScore() {
        // PLACEHOLDER
        int armEnum;

        rotationIsNeeded = true;
        goalYaw = 180; // All scoring is facing towards our drivers

        switch(currentGameObject) {
            case CONE:
                currentLimelight = limelightHigh;
                currentLimelight.setPipeline(3); // Tape pipeline
                PIDArea = new PIDController(7, 0, 0); // NOT SURE IF CORRECT, Updated 2/23/2023
                PIDTX = new PIDController(0.1, 0, 0);
                PIDYaw = new PIDController(0.1, 0, 0);
                goalArea = 0.15; // Unsure if correct, updated 2/23/2023
                break;

            case CUBE:
                currentLimelight = limelightLow;
                currentLimelight.setPipeline(4); // April tag pipeline
                PIDArea = new PIDController(1, 0, 0); // Correct PID as of 2/23/2023
                PIDTX = new PIDController(0.08, 0, .01);
                PIDYaw = new PIDController(0.1, 0, 0);
                goalArea = 2; // April tag target area, unsure if correct, updated 2/23/2023
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

        // Had to declare both RunCommands in advance because syntax errors would appear if they weren't
        RunCommand driveRotateToTargetRunCommand = new RunCommand(() -> driveRotateToTarget(PIDArea, PIDTX, PIDYaw), arm, elevator, claw, drivetrain);
        RunCommand driveToTargetRunCommand = new RunCommand(() -> skrttttToTarget(PIDArea, PIDTX), arm, elevator, claw, drivetrain);
        RunCommand currentVisionRunCommand;

        if (rotationIsNeeded) {
            currentVisionRunCommand = driveRotateToTargetRunCommand;
        } else {
            currentVisionRunCommand = driveToTargetRunCommand;
        }

        InstantCommand init = new InstantCommand(() -> initVisionCommands());

        return new SequentialCommandGroup(
            new InstantCommand(() -> SmartDashboard.putBoolean("Vision Score Running", true)),
            init,
            // Stow arm
            currentVisionRunCommand,
            // Arm to arm enum position
            // Open claw/eject piece with rollers
            // Wait 1 second
            // Stow arm
            // Close claw/stop rollers
            new InstantCommand(() -> SmartDashboard.putBoolean("Vision Score Running", false))
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

    // Mutators

    public CommandBase updateCurrentGameObject(OBJECT_TYPE oType) {
        return runOnce(
            () -> {
                currentGameObject = oType;
            });
    }

    public CommandBase updateCurrentHeight(SCORE_POS sPos) {
        return runOnce(
            () -> {
                currentHeightPos = sPos;
            });
    }

    // Smartdashboard

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
