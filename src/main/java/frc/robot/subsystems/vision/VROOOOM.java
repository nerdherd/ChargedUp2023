package frc.robot.subsystems.vision;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MotorClaw;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.Limelight.LightMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.NerdyMath;

import edu.wpi.first.wpilibj2.command.Commands;

// Subsystem integration notes from Ayaka
// arm.movearmmotionmagic(ticks, elevator.percentextended)
// elevator.movemotionmagic(ticks, arm.getarmangle)
// claw negative is intake at 30%

public class VROOOOM extends SubsystemBase implements Reportable{

    private Limelight limelightHigh;
    private Limelight limelightLow;
    private Limelight currentLimelight = null;

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
    private MotorClaw claw;
    private SwerveDrivetrain drivetrain;

    public VROOOOM(Arm arm, Elevator elevator, MotorClaw claw, SwerveDrivetrain drivetrain) {
        this.arm = arm;
        this.elevator = elevator;
        this.claw = claw;
        this.drivetrain = drivetrain;

        // defaults
        goalArea = 0;
        goalTX = 0;
        goalYaw = 0;
        rotationIsNeeded = false;

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
        
        cameraStatusSupplier = () -> (currentCameraMode == CAMERA_MODE.ARRIVED);
        //cameraConfig(limelightLow, OBJECT_TYPE.CONE, SCORE_POS.HIGH, CAMERA_MODE.IDLE);
		SmartDashboard.putNumber("Tx P", 0);       
        SmartDashboard.putNumber("Tx I", 0);
        SmartDashboard.putNumber("Tx D", 0);

        SmartDashboard.putNumber("Ta P", 0);       
        SmartDashboard.putNumber("Ta I", 0);
        SmartDashboard.putNumber("Ta D", 0);

        SmartDashboard.putNumber("Yaw P", 0);       
        SmartDashboard.putNumber("Yaw I", 0);
        SmartDashboard.putNumber("Yaw D", 0);
    }

    public void cameraConfig(Limelight l, OBJECT_TYPE t, SCORE_POS p, CAMERA_MODE m) {
        currentLimelight = l;
        currentGameObject = t;
        currentHeightPos = p;
        currentCameraMode = m;
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

     
    public void initVisionCommands( ) {
        currentCameraMode = CAMERA_MODE.IDLE;
        initDoneX = false;
        initDoneArea = false;
        xIndex = 0;
        areaIndex = 0;
    }

    public void initVisionPickupOnGround(OBJECT_TYPE objType) {
        initVisionCommands();

        //int armPositionTicks = ArmConstants.kArmStow;
        //    int elevatorPositionTicks = ElevatorConstants.kElevatorStow;

            currentGameObject = objType;
            currentHeightPos = SCORE_POS.LOW;
            rotationIsNeeded = false; // Reset rotation variable
                
            //armPositionTicks = ArmConstants.kArmGroundPickup; // Ground pickup
            //elevatorPositionTicks = ElevatorConstants.kElevatorStow;
            currentLimelight = limelightLow;
            rotationIsNeeded = false;

            if (currentGameObject == OBJECT_TYPE.CONE) {
                goalArea = 21; // Goal area for cone ground pickup, area is an estimate because a different camera position was used, updated 2/23/2023
                currentLimelight.setPipeline(1);
                PIDArea = new PIDController(SmartDashboard.getNumber("Ta P", 0), SmartDashboard.getNumber("Ta I", 0), SmartDashboard.getNumber("Ta D", 0));
                PIDTX = new PIDController(SmartDashboard.getNumber("Tx P", 0), SmartDashboard.getNumber("Tx I", 0), SmartDashboard.getNumber("Tx D", 0));
                PIDYaw = new PIDController(SmartDashboard.getNumber("Yaw P", 0), SmartDashboard.getNumber("Yaw I", 0), SmartDashboard.getNumber("Yaw D", 0));
            } else {
                goalArea = 0; // Goal area for cube ground pickup
                currentLimelight.setPipeline(2);
                PIDArea = new PIDController(SmartDashboard.getNumber("Ta P", 0), SmartDashboard.getNumber("Ta I", 0), SmartDashboard.getNumber("Ta D", 0));
                PIDTX = new PIDController(SmartDashboard.getNumber("Tx P", 0), SmartDashboard.getNumber("Tx I", 0), SmartDashboard.getNumber("Tx D", 0));
                PIDYaw = new PIDController(SmartDashboard.getNumber("Yaw P", 0), SmartDashboard.getNumber("Yaw I", 0), SmartDashboard.getNumber("Yaw D", 0));
            }
    
            //final int armPositionTicksFinal  = armPositionTicks;
            //final int elevatorPositionTicksFinal  = elevatorPositionTicks;
    }
    public CommandBase VisionPickupOnGround(OBJECT_TYPE objType) {
        if(limelightLow != null) {
            // Had to declare both RunCommands in advance because syntax errors would appear if they weren't
            // RunCommand driveRotateToTargetRunCommand = new RunCommand(() -> driveRotateToTarget(PIDArea, PIDTX, PIDYaw), arm, elevator, claw, drivetrain);
            // RunCommand driveToTargetRunCommand = new RunCommand(() -> skrttttToTarget(PIDArea, PIDTX), arm, elevator, claw, drivetrain);
            // RunCommand currentVisionRunCommand;
    
            // if (rotationIsNeeded) {
            //     currentVisionRunCommand = driveRotateToTargetRunCommand;
            // } else {
            //     currentVisionRunCommand = driveToTargetRunCommand;
            // }
    
            return Commands.parallel(
                // Constantly run elevator and arm motion magic
                elevator.moveElevator(arm::getArmAngle),
                arm.moveArm(elevator::percentExtended),

                Commands.sequence(
                    Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", true)),
                    Commands.runOnce(() -> initVisionPickupOnGround(objType)),
    
                    // Arm and elevator to selected position
                    Commands.race(
                        Commands.waitSeconds(5),
                        Commands.parallel( // End command once both arm and elevator have reached their target position
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup)),
                            Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow))
                        )
                    ),
                    
                    new RunCommand(() -> driveRotateToTarget(PIDArea, PIDTX, PIDYaw), arm, elevator, claw, drivetrain).until(cameraStatusSupplier).withTimeout(5), // Timeout after 30 seconds
    
                    // Open claw/Start claw intake rollers
                    claw.setPower(-0.3),
                    new WaitCommand(2),
    
                    // Close claw/stop claw intake rollers/low background rolling to keep control of game piece
                    claw.setPower(-0.15),
    
                    // Stow arm/elev
                    Commands.race(
                        Commands.waitSeconds(5),
                        Commands.parallel( // End command once both arm and elevator have reached their target position
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                            Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow))
                        )
                    ),
                    
                    Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", false))
                )
            );
            
        }
        else {
            return runOnce(() -> SmartDashboard.putString("Limelight command status:", "Sequence cancelled"));
        }
    }

    public void initVisionPickupOnSubstation(OBJECT_TYPE objType) {
        initVisionCommands();
        //int armPositionTicks = ArmConstants.kArmStow;
            //int elevatorPositionTicks = ElevatorConstants.kElevatorStow;

            currentGameObject = objType;
    
            rotationIsNeeded = false; // Reset rotation variable

            currentHeightPos = SCORE_POS.HIGH;
    
            //armPositionTicks = ArmConstants.kArmSubstation; // Pickup substation
            //elevatorPositionTicks = ElevatorConstants.kElevatorSubstation;
            currentLimelight = limelightHigh;
            rotationIsNeeded = true;
            goalYaw = 0; // Facing away from drivers, towards substation

            if (currentGameObject == OBJECT_TYPE.CONE) {
                goalArea = 21; // Goal area for cone substation pickup, area is an estimate because a different camera position was used, updated 2/23/2023
                currentLimelight.setPipeline(1);
                PIDArea = new PIDController(SmartDashboard.getNumber("Ta P", 0), SmartDashboard.getNumber("Ta I", 0), SmartDashboard.getNumber("Ta D", 0));
                PIDTX = new PIDController(SmartDashboard.getNumber("Tx P", 0), SmartDashboard.getNumber("Tx I", 0), SmartDashboard.getNumber("Tx D", 0));
                PIDYaw = new PIDController(SmartDashboard.getNumber("Yaw P", 0), SmartDashboard.getNumber("Yaw I", 0), SmartDashboard.getNumber("Yaw D", 0));
            } else {
                goalArea = 0; // Goal area for cube substation pickup
                currentLimelight.setPipeline(2);
                PIDArea = new PIDController(SmartDashboard.getNumber("Ta P", 0), SmartDashboard.getNumber("Ta I", 0), SmartDashboard.getNumber("Ta D", 0));
                PIDTX = new PIDController(SmartDashboard.getNumber("Tx P", 0), SmartDashboard.getNumber("Tx I", 0), SmartDashboard.getNumber("Tx D", 0));
                PIDYaw = new PIDController(SmartDashboard.getNumber("Yaw P", 0), SmartDashboard.getNumber("Yaw I", 0), SmartDashboard.getNumber("Yaw D", 0));
            }

            //final int armPositionTicksFinal  = armPositionTicks;
            //final int elevatorPositionTicksFinal  = elevatorPositionTicks;
        }
    public CommandBase VisionPickupOnSubstation(OBJECT_TYPE objType) {
        if(limelightHigh != null) {
            
    
            // Had to declare both RunCommands in advance because syntax errors would appear if they weren't
            //RunCommand driveRotateToTargetRunCommand = new RunCommand(() -> driveRotateToTarget(PIDArea, PIDTX, PIDYaw), arm, elevator, claw, drivetrain);
            //RunCommand driveToTargetRunCommand = new RunCommand(() -> skrttttToTarget(PIDArea, PIDTX), arm, elevator, claw, drivetrain);
            //RunCommand currentVisionRunCommand;
    
            //if (rotationIsNeeded) {
            //    currentVisionRunCommand = driveRotateToTargetRunCommand;
            //} else {
            //    currentVisionRunCommand = driveToTargetRunCommand;
            //}
    
            return Commands.parallel(
                // Constantly run elevator and arm motion magic
                //elevator.moveElevator(arm::getArmAngle),
                //arm.moveArm(elevator::percentExtended),

                Commands.sequence(
                    Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", true)),
                    Commands.runOnce(() -> initVisionPickupOnSubstation(objType)),
    
                    // Arm and elevator to selected position
                    /*Commands.race(
                        Commands.waitSeconds(5),
                        Commands.parallel( // End command once both arm and elevator have reached their target position
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmSubstation)),
                            Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorSubstation))
                        )
                    ),*/
                    
                    new RunCommand(() -> driveRotateToTarget(PIDArea, PIDTX, PIDYaw), arm, elevator, claw, drivetrain).until(cameraStatusSupplier).withTimeout(5), // Timeout after 30 seconds
    
                    // Open claw/Start claw intake rollers
                    claw.setPower(-0.3),
                    new WaitCommand(2),
    
                    // Close claw/stop claw intake rollers/low background rolling to keep control of game piece
                    claw.setPower(-0.15),
    
                    // Stow arm/elev
                    Commands.race(
                        Commands.waitSeconds(5),
                        Commands.parallel( // End command once both arm and elevator have reached their target position
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                            Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow))
                        )
                    ),
                    
                    Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", false))
                )
            );
        }
        else {
            return runOnce(() -> SmartDashboard.putString("Limelight command status:", "Sequence cancelled"));
        }
    }

    
    int armPositionTicks = ArmConstants.kArmStow;
    int elevatorPositionTicks = ElevatorConstants.kElevatorStow;
    public void initVisionScore(OBJECT_TYPE objType, SCORE_POS pos) {

        initVisionCommands();

        currentGameObject = objType;
        currentHeightPos = pos;
        rotationIsNeeded = true;
        goalYaw = 180; // All scoring is facing towards our drivers

        switch(currentGameObject) {
            case CONE:
                currentLimelight = limelightLow; // Use low for testing
                currentLimelight.setPipeline(3); // Tape pipeline
                currentLimelight.setLightState(LightMode.ON);
                PIDArea = new PIDController(SmartDashboard.getNumber("Ta P", 0), SmartDashboard.getNumber("Ta I", 0), SmartDashboard.getNumber("Ta D", 0));
                PIDTX = new PIDController(SmartDashboard.getNumber("Tx P", 0), SmartDashboard.getNumber("Tx I", 0), SmartDashboard.getNumber("Tx D", 0));
                PIDYaw = new PIDController(SmartDashboard.getNumber("Yaw P", 0), SmartDashboard.getNumber("Yaw I", 0), SmartDashboard.getNumber("Yaw D", 0));
                goalArea = 0.15; // Unsure if correct, updated 2/23/2023

                switch(currentHeightPos) {
                    case HIGH:
                        armPositionTicks = ArmConstants.kArmScore; // Score high
                        elevatorPositionTicks = ElevatorConstants.kElevatorScoreHigh;
                        break;
        
                    case MID:
                        
                        armPositionTicks = ArmConstants.kArmScore; // Score mid
                        elevatorPositionTicks = ElevatorConstants.kElevatorScoreMid;
                        break;
        
                    case LOW: // Score ground
                        armPositionTicks = ArmConstants.kArmGroundPickup;
                        elevatorPositionTicks = ElevatorConstants.kElevatorStow;
                        break;
                }
                break;

            case CUBE:
                currentLimelight = limelightLow;
                currentLimelight.setPipeline(4); // April tag pipeline
                PIDArea = new PIDController(SmartDashboard.getNumber("Ta P", 0), SmartDashboard.getNumber("Ta I", 0), SmartDashboard.getNumber("Ta D", 0));
                PIDTX = new PIDController(SmartDashboard.getNumber("Tx P", 0), SmartDashboard.getNumber("Tx I", 0), SmartDashboard.getNumber("Tx D", 0));
                PIDYaw = new PIDController(SmartDashboard.getNumber("Yaw P", 0), SmartDashboard.getNumber("Yaw I", 0), SmartDashboard.getNumber("Yaw D", 0));
                goalArea = 2; // April tag target area, unsure if correct, updated 2/23/2023

                switch(currentHeightPos) {
                    case HIGH:
                        armPositionTicks = ArmConstants.kArmScore; // Score high
                        elevatorPositionTicks = ElevatorConstants.kElevatorScoreHigh;
                        break;
        
                    case MID:
                        
                        armPositionTicks = ArmConstants.kArmScore; // Score mid
                        elevatorPositionTicks = ElevatorConstants.kElevatorScoreMid;
                        break;
        
                    case LOW: // Score ground
                        armPositionTicks = ArmConstants.kArmGroundPickup;
                        elevatorPositionTicks = ElevatorConstants.kElevatorStow;
                        break;
                }

                break;
        }

        

        

        //final int armPositionTicksFinal  = armPositionTicks;
        //final int elevatorPositionTicksFinal  = elevatorPositionTicks;
    }

    public CommandBase VisionScore(OBJECT_TYPE objType, SCORE_POS pos) {
        //if(currentLimelight != null) {
            // if (currentLimelight)
            // SmartDashboard.putBoolean("Vision has target", currentLimelight.hasValidTarget());
            
    
            // Had to declare both RunCommands in advance because syntax errors would appear if they weren't
            // RunCommand driveRotateToTargetRunCommand = new RunCommand(() -> driveRotateToTarget(PIDArea, PIDTX, PIDYaw), arm, elevator, claw, drivetrain);
            // RunCommand driveToTargetRunCommand = new RunCommand(() -> skrttttToTarget(PIDArea, PIDTX), arm, elevator, claw, drivetrain);
            // RunCommand currentVisionRunCommand;
    
            // if (rotationIsNeeded) {
            //     currentVisionRunCommand = driveRotateToTargetRunCommand;
            // } else {
            //     currentVisionRunCommand = driveToTargetRunCommand;
            // }
            return Commands.parallel(
                // Constantly run elevator and arm motion magic
                //elevator.moveElevator(arm::getArmAngle),
                //arm.moveArm(elevator::percentExtended),
                
                Commands.sequence(
                    // Commands.parallel(
                    //     Commands.runOnce(() -> SmartDashboard.putString("Vision Score Stage", "Stow")),
                    //     Commands.runOnce(() -> initVisionScore(objType, pos))
                    // ),
                    
                    // // Stow arm
                    // Commands.race(
                    //     Commands.waitSeconds(5),
                    //     Commands.parallel( // End command once both arm and elevator have reached their target position
                    //         Commands.waitUntil(arm.atTargetPosition),
                    //         Commands.waitUntil(elevator.atTargetPosition),
                    //         Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                    //         Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow))
                    //     )
                    // ),
                    
                    new RunCommand(() -> driveRotateToTarget(PIDArea, PIDTX, PIDYaw), arm, elevator, claw, drivetrain).until(cameraStatusSupplier).withTimeout(5),
    
                    // Arm and elevator to selected position
                    // Commands.race(
                    //     Commands.waitSeconds(5),
                    //     Commands.parallel( // End command once both arm and elevator have reached their target position
                    //         Commands.waitUntil(arm.atTargetPosition),
                    //         Commands.waitUntil(elevator.atTargetPosition),
                    //         Commands.runOnce(() -> arm.setTargetTicks(armPositionTicks)),
                    //         Commands.runOnce(() -> elevator.setTargetTicks(elevatorPositionTicks))
                    //     )
                    // ),
                    
                    // Open claw/eject piece with rollers
                    // claw.setPower(1),
                    // // Wait 1 second
                    // Commands.waitSeconds(1),
    
                    // // Close claw/stop rollers
                    // claw.setPower(0),
    
                    // Stow arm
                    // Commands.race(
                    //     Commands.waitSeconds(5),
                    //     Commands.parallel( // End command once both arm and elevator have reached their target position
                    //         Commands.waitUntil(arm.atTargetPosition),
                    //         Commands.waitUntil(elevator.atTargetPosition),
                    //         Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                    //         Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow))
                    //     )
                    // ),
                    
                    Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Score Running", false))
                )
            );
            
        // }
        // else {
        //     return runOnce(() -> SmartDashboard.putString("Limelight command status:", "Sequence cancelled"));
        // }
        
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
            rotationSpeed = pidYaw.calculate(drivetrain.getGyro().getHeading(), goalYaw);
            
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
    private void skrttttToTarget(PIDController pidArea, PIDController pidTX) {
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

    /*public CommandBase updateCurrentGameObject(OBJECT_TYPE oType) {
        return runOnce(
            () -> {
                currentGameObject = oType;
            });
    }*/

    // public CommandBase updateCurrentHeight(SCORE_POS sPos) {
    //     return runOnce(
    //         () -> {
    //             currentHeightPos = sPos;
    //         });
    // }

    // Smartdashboard

    @Override
    public void reportToSmartDashboard() {
        if( currentGameObject != null)
        SmartDashboard.putString("Vision Current Object", currentGameObject.toString());
        if(currentHeightPos != null)
        SmartDashboard.putString("Vision Current Height", currentHeightPos.toString());
        if(currentLimelight != null) {
            SmartDashboard.putString("Vision Current Limelight", currentLimelight.getName());
            SmartDashboard.putNumber("Vision Current Pipeline", currentLimelight.getPipeIndex());
        }
        else {
            SmartDashboard.putString("Vision Current Limelight", "L + ratio");
        }
    }

    @Override
    public void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab(this.getName());
        if( currentGameObject != null)
        tab.addString("Vision Current Object", () -> currentGameObject.toString());
        if(currentHeightPos != null)
        tab.addString("Vision Current Height", () -> currentHeightPos.toString());
        if(currentLimelight != null) {
            tab.addBoolean("has target", currentLimelight::hasValidTarget);
            tab.addString("Vision Current Limelight", () -> currentLimelight.getName());
            tab.addNumber("Vision Pipeline", () -> currentLimelight.getPipeIndex());
        }
        else {
            tab.addString("Vision Current Limelight", () -> "L + ratio");
        }
    }
}
