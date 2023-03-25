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
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.claw.MotorClaw;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.Limelight.LightMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.NerdyMath;

import edu.wpi.first.wpilibj2.command.Commands;

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
            limelightLow.setLightState(Limelight.LightMode.ON);
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

    // Returns ID of AprilTag if one is in front of the robot when this command is called. Otherwise, returns -1.
    public int getAprilTagID() {
        if (limelightLow != null) {
            limelightLow.setPipeline(4); // April tag pipeline
            if (limelightLow.hasValidTarget()) {
                return limelightLow.getAprilTagID();
            }
        }

        return -1;
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

        currentGameObject = objType;
        currentHeightPos = SCORE_POS.LOW;
        rotationIsNeeded = false; // Reset rotation variable
        currentLimelight = limelightLow;

        // This doesn't work for some reason, so we might need to pass the currentGameObject into the drive command directly. (3/11/2023)
        if (currentGameObject == OBJECT_TYPE.CONE) {
            goalArea = 2.8; // Alex changed from 3.4 to 2.6 //3.8; // This line is running, so we know the conditional is working (3/11/2023)
            currentLimelight.setPipeline(1);

            // Old PID for max 4 m/s
            PIDArea.setPID(0.4, 0.01, 0.01);
            PIDTX.setPID(0.05, 0.01, 0.01);
            PIDYaw.setPID(0, 0, 0);

            // New PID for max 5 m/s
            // PIDArea.setPID(0.5, 0, 0.0125);
            // PIDTX.setPID(0.05, 0, 0.0125);
            // PIDYaw.setPID(0, 0, 0);
        } else {
            goalArea = 2.3; // Goal area for cube ground pickup
            currentLimelight.setPipeline(2);
            currentLimelight.setLightState(LightMode.OFF);

            // PIDArea.setPID(
            //         SmartDashboard.getNumber("Ta P", 0.75),
            //         SmartDashboard.getNumber("Ta I", 0.0),
            //         SmartDashboard.getNumber("Ta D", 0.02)
            //     );
            //     PIDTX.setPID(
            //         SmartDashboard.getNumber("Tx P", 0.05),
            //         SmartDashboard.getNumber("Tx I", 0.0),
            //         SmartDashboard.getNumber("Tx D", 0.008)
            //     );

            PIDArea.setPID(0.75, 0, 0.02);
            PIDTX.setPID(0.05, 0, 0.008);
            PIDYaw.setPID(0, 0, 0);
        }
    }

    public CommandBase VisionPickupOnGround(OBJECT_TYPE objType) {
        final PIDController pidAreaFinal = PIDArea;
        final PIDController pidTXFinal = PIDTX;
        final PIDController pidYawFinal = PIDYaw;

        if(limelightLow != null) {
            return Commands.race(
                // Constantly run elevator and arm motion magic
                Commands.run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
                Commands.run(() -> elevator.moveMotionMagic(arm.getArmAngle())),

                Commands.sequence(
                    Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", true)),
                    Commands.runOnce(() -> initVisionPickupOnGround(objType)),
    
                    // Move arm and elevator to near ground position in parallel with approaching target
                    Commands.deadline(
                        Commands.waitSeconds(2),
                        Commands.parallel( // End command once both arm and elevator have reached their target position
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.runOnce(() -> arm.setTargetTicks(-328500)),
                            Commands.runOnce(() -> elevator.setTargetTicks(-36000))
                        ),
                        new RunCommand(() -> driveRotateToTarget(pidAreaFinal, pidTXFinal, pidYawFinal), arm, elevator, claw, drivetrain).until(cameraStatusSupplier)
                    ),
                    
    
                    // Drop arm and elevator so the game piece can be intook
                    Commands.race(
                        Commands.waitSeconds(5),
                        Commands.parallel( // End command once both arm and elevator have reached their target position
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup)),
                            Commands.runOnce(() -> elevator.setTargetTicks(-160000))
                        )
                    ),

                    // Open claw/Start claw intake rollers
                    claw.setPower(-0.3),
                    new WaitCommand(.5),
    
                    // // Close claw/stop claw intake rollers/low background rolling to keep control of game piece
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

    public CommandBase VisionPickupGroundNoArm(OBJECT_TYPE objType) {
        final PIDController pidAreaFinal = PIDArea;
        final PIDController pidTXFinal = PIDTX;
        final PIDController pidYawFinal = PIDYaw;

        if(limelightLow != null) {
            return Commands.race(
                // Constantly run elevator and arm motion magic
                Commands.run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
                Commands.run(() -> elevator.moveMotionMagic(arm.getArmAngle())),

                Commands.sequence(
                    Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", true)),
                    Commands.runOnce(() -> initVisionPickupOnGround(objType)),

                    Commands.race(
                        new RunCommand(() -> driveRotateToTarget(pidAreaFinal, pidTXFinal, pidYawFinal), arm, elevator, claw, drivetrain).until(cameraStatusSupplier),
                        Commands.waitSeconds(2)
                    ),
    
                    // Drop arm and elevator so the game piece can be intook
                    Commands.race(
                        Commands.waitSeconds(5),
                        Commands.parallel( // End command once both arm and elevator have reached their target position
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup)),
                            Commands.sequence(
                                Commands.waitSeconds(0.25),
                                Commands.runOnce(() -> elevator.setTargetTicks(-160000))
                            )
                        )
                    ),

                    // Open claw/Start claw intake rollers
                    claw.setPower(-0.3),
                    new WaitCommand(.5),
    
                    // // Close claw/stop claw intake rollers/low background rolling to keep control of game piece
                    claw.setPower(-0.15),
                    
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

                PIDArea.setPID(
                    SmartDashboard.getNumber("Ta P", 0.4),
                    SmartDashboard.getNumber("Ta I", 0.01),
                    SmartDashboard.getNumber("Ta D", 0.01)
                );
                PIDTX.setPID(
                    SmartDashboard.getNumber("Tx P", 0.04),
                    SmartDashboard.getNumber("Tx I", 0.01),
                    SmartDashboard.getNumber("Tx D", 0.01)
                );
                
                // PIDArea.setPID(0.4, 0.01, 0.01);
                // PIDTX.setPID(0.04, 0.01, 0.01);
                PIDYaw.setPID(0, 0, 0);
            } else {
                goalArea = 0; // Goal area for cube substation pickup
                currentLimelight.setPipeline(2);
                
                PIDArea.setPID(0, 0, 0);
                PIDTX.setPID(0, 0, 0);
                PIDYaw.setPID(0, 0, 0);
            }

            //final int armPositionTicksFinal  = armPositionTicks;
            //final int elevatorPositionTicksFinal  = elevatorPositionTicks;
        }
    public CommandBase VisionPickupOnSubstation(OBJECT_TYPE objType) {
        if(limelightHigh != null) {

            return Commands.race(
                // Constantly run elevator and arm motion magic
                // run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
                // run(() -> elevator.moveMotionMagic(arm.getArmAngle())),

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
        goalYaw = 0; // All scoring is facing towards our drivers

        switch(currentGameObject) {
            case CONE:
                currentLimelight = limelightLow; // Use low for testing
                currentLimelight.setPipeline(3); // Tape pipeline
                currentLimelight.setLightState(LightMode.ON);

                // PID when the max speed was 4 m/s
                PIDArea.setPID(2, 0, 0);
                PIDTX.setPID(0.08, 0.02, 0.02);
                PIDYaw.setPID(0, 0, 0);

                // New PID for max speed of 5 m/s, just calculated (multiplied by 5/4) but has to be tuned
                // PIDArea.setPID(2.5, 0, 0);
                // PIDTX.setPID(0.1, 0, 0.025);
                // PIDYaw.setPID(0, 0, 0);

                goalArea = 0.55;

                switch(currentHeightPos) {
                    case HIGH:
                        armPositionTicks = ArmConstants.kArmScore - 15900; // Score high
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
                
                PIDArea.setPID(SmartDashboard.getNumber("Ta P", 0), SmartDashboard.getNumber("Ta I", 0), SmartDashboard.getNumber("Ta D", 0));
                PIDTX.setPID(SmartDashboard.getNumber("Tx P", 0), SmartDashboard.getNumber("Tx I", 0), SmartDashboard.getNumber("Tx D", 0));
                // PIDYaw.setPID(10, 0, 0.2);
                PIDYaw.setPID(0, 0, 0);

                goalArea = 7.2; // April tag target area, unsure if correct, updated 2/23/2023

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
    }

    public CommandBase VisionScore(OBJECT_TYPE objType, SCORE_POS pos) {
        final PIDController pidAreaFinal = PIDArea;
        final PIDController pidTXFinal = PIDTX;
        final PIDController pidYawFinal = PIDYaw;

        if (limelightLow != null) {
            return Commands.race(
                // Constantly run elevator and arm motion magic
                Commands.run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
                Commands.run(() -> elevator.moveMotionMagic(arm.getArmAngle())),
                
                Commands.sequence(
                    Commands.parallel(
                        Commands.runOnce(() -> SmartDashboard.putString("Vision Score Stage", "Stow")),
                        Commands.runOnce(() -> initVisionScore(objType, pos))
                    ),
                    
                    new TurnToAngle(0, drivetrain), //  TODO: merge with driveRotateToTarget Yaw PID
    
                    Commands.parallel(
                        new RunCommand(() -> driveRotateToTarget(pidAreaFinal, pidTXFinal, pidYawFinal), arm, elevator, claw, drivetrain)
                            .until(cameraStatusSupplier)
                            .withTimeout(2),
                            
                        // Move arm and elevator, arm is moved 0.5 seconds after the elevator to prevent power chain from getting caught
                        Commands.race(
                            Commands.waitSeconds(5), // Timeout
                            Commands.sequence(
                                Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScore)),
                                Commands.waitSeconds(0.5),
                                
                                Commands.parallel( // End when target positions reached
                                    Commands.waitUntil(elevator.atTargetPosition),
                                    Commands.waitUntil(arm.atTargetPosition),
                                    Commands.runOnce(() -> elevator.setTargetTicks(elevatorPositionTicks))
                                )
                            )
                        )
                    ),
                    
                    new TurnToAngle(0, drivetrain),

                    Commands.waitSeconds(0.5),
                    // Open claw/eject piece with rollers
                    claw.setPower(1),
    
                    // Wait to outtake
                    Commands.waitSeconds(.5),
    
                    // Close claw/stop rollers
                    claw.setPower(0),

                    // Stow arm
                    Commands.race(
                        Commands.waitSeconds(5),
                        Commands.parallel( // End command once both arm and elevator have reached their target position
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                            Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow))
                        )
                    ),
    
                    // new TurnToAngle(0, drivetrain), // Turn back towards field after scoring
                    Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Score Running", false))
                )
            );
        }
        else {
            return runOnce(() -> SmartDashboard.putString("Limelight command status:", "Sequence cancelled"));
        }
    }

    // Moves arm, but doesn't stow after vision align
    public CommandBase VisionScoreNoArm(OBJECT_TYPE objType, SCORE_POS pos) {
        final PIDController pidAreaFinal = PIDArea;
        final PIDController pidTXFinal = PIDTX;
        final PIDController pidYawFinal = PIDYaw;

        if (limelightLow != null) {
            return Commands.race(
                // Constantly run elevator and arm motion magic
                Commands.run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
                Commands.run(() -> elevator.moveMotionMagic(arm.getArmAngle())),
                
                Commands.sequence(
                    Commands.parallel(
                        Commands.runOnce(() -> SmartDashboard.putString("Vision Score Stage", "Stow")),
                        Commands.runOnce(() -> initVisionScore(objType, pos))
                    ),
                    
                    Commands.parallel(
                        new RunCommand(() -> driveRotateToTarget(pidAreaFinal, pidTXFinal, pidYawFinal), arm, elevator, claw, drivetrain)
                            .until(cameraStatusSupplier)
                            .withTimeout(0.5),
                            
                        // Move arm and elevator, arm is moved 0.5 seconds after the elevator to prevent power chain from getting caught
                        Commands.race(
                            Commands.waitSeconds(5), // Timeout
                            Commands.sequence(
                                Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScore)),
                                // Commands.waitSeconds(0.5),
                                
                                Commands.parallel( // End when target positions reached
                                    Commands.waitUntil(elevator.atTargetPosition),
                                    Commands.waitUntil(arm.atTargetPosition),
                                    Commands.runOnce(() -> elevator.setTargetTicks(elevatorPositionTicks))
                                )
                            )
                        )
                    ),

                    Commands.waitSeconds(0.5),
                    // Open claw/eject piece with rollers
                    claw.setPower(1),
    
                    // Wait to outtake
                    Commands.waitSeconds(.5),
    
                    // Close claw/stop rollers
                    claw.setPower(0),
    
                    // new TurnToAngle(0, drivetrain), // Turn back towards field after scoring
                    Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Score Running", false))
                )
            );
        }
        else {
            return runOnce(() -> SmartDashboard.putString("Limelight command status:", "Sequence cancelled"));
        }
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
            // NOTE (3/11/2023): This should be commented out, BUT if the PID is still 0,
            // you can use this but we still need to find a permenant solution

            // Score cone (low tape, max distance is the charging station)
            // pidArea.setP(2);
            // pidArea.setD(0);
            // pidTX.setP(0.08);
            // pidTX.setD(0.02);

            // Cone ground
            // pidArea.setP(0.4);
            // pidArea.setD(0.01);
            // pidTX.setP(0.04);
            // pidTX.setD(0.01);

            double calculatedX = getAvgArea(currentLimelight.getArea_avg());
            double calculatedY = getAvgTX(currentLimelight.getXAngle_avg());
            SmartDashboard.putNumber("Vision average X", calculatedX);
            SmartDashboard.putNumber("Vision average Y", calculatedY);

            if(currentLimelight.getPipeIndex()==4){
                if (NerdyMath.inRange(calculatedY, -2.2, 1) 
                    && calculatedX > 7) {
                chassisSpeeds = new ChassisSpeeds(0, 0, 0);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                currentCameraMode = CAMERA_MODE.ARRIVED; 
                return;
                }
            }
            xSpeed = pidArea.calculate(calculatedX, goalArea) * (5/4);
            ySpeed = -pidTX.calculate(calculatedY, goalTX) * (5/4);
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
    public void reportToSmartDashboard(LOG_LEVEL level) {
        switch (level) {
            case OFF:
                break;
            case ALL:
            case MEDIUM:
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
            case MINIMAL:
                break;
        }
    }

    @Override
    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF || level == LOG_LEVEL.MINIMAL)  {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab(this.getName());
        switch (level) {
            case OFF:
                break;
            case ALL:
            case MEDIUM:
                tab.addString("Vision Current Object", () -> {
                    if (currentGameObject != null) {
                        return currentGameObject.toString();
                    }
                    return "";
                });
                tab.addString("Vision Current Height", () -> {
                    if(currentHeightPos != null) {
                        return currentHeightPos.toString();
                    } 
                    return "";
                });
                tab.addBoolean("has target", () -> {
                    if(currentLimelight != null)  {
                        return currentLimelight.hasValidTarget();
                    }
                    return false;
                });
                tab.addNumber("Pipeline", () -> {
                    if(currentLimelight != null)  {
                        return currentLimelight.getPipeIndex();
                    }
                    return -1;
                });
                tab.addString("Current Limelight", () -> {
                    if(currentLimelight != null)  {
                        return currentLimelight.getName();
                    }
                    return "L + ratio";
                });
            case MINIMAL:
                break;
        }
    }
}
