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
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.MotorClaw;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.Limelight.LightMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.NerdyMath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.Timer;


public class AHealthyPairOfEyes extends SubsystemBase implements Reportable{

    private Limelight limelightLeft;
    private Limelight limelightRight;

    // Pipelines that should be set on each limelight: cone = 1, cube = 2, tape = 3, tag = 4

    public enum OBJECT_TYPE
    {
        CONE,
        CUBE,
        TAPE,
        ATAG
    }
    
    public enum CAMERA_MODE
    {
        WAIT, // found nothing
        IDLE, // doing nothing, init
        ACTION,// detected one, and approaching to it
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
    public BooleanSupplier cameraStatusSupplier;


    // Current PID Controllers
    private PIDController PIDArea = new PIDController(0, 0, 0);
    private PIDController PIDTX = new PIDController(0, 0, 0);
    private PIDController PIDYaw = new PIDController(0, 0, 0);
    
    private Arm arm;
    private Elevator elevator;
    private MotorClaw claw;
    private SwerveDrivetrain drivetrain;

    public AHealthyPairOfEyes(Arm arm, Elevator elevator, MotorClaw claw, SwerveDrivetrain drivetrain) {
        this.arm = arm;
        this.elevator = elevator;
        this.claw = claw;
        this.drivetrain = drivetrain;

        // defaults
        goalArea = 0;
        goalTX = 0;
        goalYaw = 0;

        try {
            limelightLeft = new Limelight("limelight-left");
            limelightLeft.setLightState(Limelight.LightMode.OFF);
        } catch (Exception ex) {
            limelightLeft = null;
            DriverStation.reportWarning("Error instantiating left camera:  " + ex.getMessage(), true);
        }

        try {
            limelightRight = new Limelight("limelight-right");
            limelightRight.setLightState(Limelight.LightMode.OFF);
        } catch (Exception ex) {
            limelightRight = null;
            DriverStation.reportWarning("Error instantiating right camera:  " + ex.getMessage(), true);
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

    public void cameraConfig(Limelight left, Limelight right, OBJECT_TYPE t, SCORE_POS p, CAMERA_MODE m) {
        limelightLeft = left;
        limelightRight = right;
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
        if (limelightRight != null) {
            limelightRight.setPipeline(4); // April tag pipeline
            if (limelightRight.hasValidTarget()) {
                return limelightRight.getAprilTagID();
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

    Timer timer = new Timer();

    public void initVisionPickupOnGround(OBJECT_TYPE objType) {
        initVisionCommands();

        timer.reset();
        timer.start();

        currentGameObject = objType;
        currentHeightPos = SCORE_POS.LOW;
        limelightLeft.setLightState(LightMode.OFF);
        limelightRight.setLightState(LightMode.OFF);

        // This doesn't work for some reason, so we might need to pass the currentGameObject into the drive command directly. (3/11/2023)
        if (currentGameObject == OBJECT_TYPE.CONE) {
            goalArea = 2.8; // Alex changed from 3.4 to 2.6 //3.8; // This line is running, so we know the conditional is working (3/11/2023)
            limelightLeft.setPipeline(1);
            limelightRight.setPipeline(1);

            // Old PID for max 4 m/s
            PIDArea.setPID(0.4, 0.01, 0.01);
            PIDTX.setPID(0.05, 0.01, 0.01);
            PIDYaw.setPID(0, 0, 0);

            // New PID for max 5 m/s
            // PIDArea.setPID(0.5, 0, 0.0125);
            // PIDTX.setPID(0.05, 0, 0.0125);
            // PIDYaw.setPID(0, 0, 0);
        } else if (currentGameObject == OBJECT_TYPE.CUBE) {
            goalArea = 4.1; // Goal area for cube ground pickup // 4.2 OG TODO: DA VINCI RED SIDE WAS CHANGED TO 4.2
            limelightLeft.setPipeline(2);
            limelightRight.setPipeline(2);

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
        else if (currentGameObject == OBJECT_TYPE.ATAG) {
            goalArea = 3.8; 
            limelightLeft.setPipeline(4);
            limelightRight.setPipeline(4);
        }
        else {
            PIDArea.setPID(0, 0, 0);
            PIDTX.setPID(0, 0, 0);
            PIDYaw.setPID(0, 0, 0);
        }
    }

    public CommandBase VisionPickupOnGround(OBJECT_TYPE objType) {
        final PIDController pidAreaFinal = PIDArea;
        final PIDController pidTXFinal = PIDTX;
        final PIDController pidYawFinal = PIDYaw;

        if((limelightRight != null)&&(limelightLeft != null)) {
            return Commands.race(
                // Constantly run elevator and arm motion magic
                // Commands.run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
                // Commands.run(() -> elevator.moveMotionMagic(arm.getArmAngle())),

                Commands.sequence(
                    Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", true)),
                    Commands.runOnce(() -> initVisionPickupOnGround(objType)),
    
                    // Move arm and elevator to near ground position in parallel with approaching target
                    // Commands.deadline(
                    //     Commands.waitSeconds(2),
                    //     Commands.parallel( // End command once both arm and elevator have reached their target position
                    //         Commands.waitUntil(arm.atTargetPosition),
                    //         Commands.waitUntil(elevator.atTargetPosition),
                    //         Commands.runOnce(() -> arm.setTargetTicks(-328500)),
                    //         Commands.runOnce(() -> elevator.setTargetTicks(-36000))
                    //     ),
                    //     new RunCommand(() -> driveRotateToTarget(pidAreaFinal, pidTXFinal, pidYawFinal), arm, elevator, claw, drivetrain).until(cameraStatusSupplier)
                    // ),
                    
    
                    // Drop arm and elevator so the game piece can be intook
                    // Commands.race(
                    //     Commands.waitSeconds(5),
                    //     Commands.parallel( // End command once both arm and elevator have reached their target position
                    //         Commands.waitUntil(arm.atTargetPosition),
                    //         Commands.waitUntil(elevator.atTargetPosition),
                    //         Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup)),
                    //         Commands.runOnce(() -> elevator.setTargetTicks(-160000))
                    //     )
                    // ),

                    // Open claw/Start claw intake rollers
                    claw.setPower(-0.3),
                    new WaitCommand(.5),
    
                    // // Close claw/stop claw intake rollers/low background rolling to keep control of game piece
                    claw.setPower(-0.15),
    
                    // Stow arm/elev
                    // Commands.race(
                    //     Commands.waitSeconds(5),
                    //     Commands.parallel( // End command once both arm and elevator have reached their target position
                    //         Commands.waitUntil(arm.atTargetPosition),
                    //         Commands.waitUntil(elevator.atTargetPosition),
                    //         Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                    //         Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow))
                    //     )
                    // ),
                    
                    Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", false))
                )
            );
            
        }
        else {
            return runOnce(() -> SmartDashboard.putString("Limelight command status:", "Sequence cancelled"));
        }
    }

    public CommandBase VisionPickupGroundNoArm(OBJECT_TYPE objType, MotorClaw claw) {
        final PIDController pidAreaFinal = PIDArea;
        final PIDController pidTXFinal = PIDTX;
        final PIDController pidYawFinal = PIDYaw;

        if((limelightLeft != null)&&(limelightRight != null))
        {

            double xSpeed = 0;
            double ySpeed = 0;
            double rotationSpeed = 0;

            SmartDashboard.putBoolean("Limelight Left has target", limelightLeft.hasValidTarget());
            SmartDashboard.putBoolean("Limelight Right has target", limelightRight.hasValidTarget());

            double leftCalculatedX = limelightLeft.getArea_avg();
            double leftCalculatedY = limelightLeft.getXAngle_avg();
            
            double rightCalculatedX = limelightRight.getArea_avg();
            double rightCalculatedY = limelightRight.getXAngle_avg();
            

            return
                Commands.sequence(
                Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", true)),
                Commands.runOnce(() -> initVisionPickupOnGround(objType)),

                Commands.runOnce(() -> SmartDashboard.putNumber("Ta Left", limelightLeft.getArea())),
                Commands.runOnce(() -> SmartDashboard.putNumber("Ta Right", limelightRight.getArea())),
                Commands.runOnce(() -> SmartDashboard.putNumber("Ta Combined", limelightLeft.getArea() + limelightRight.getArea())),

                Commands.runOnce(() -> SmartDashboard.putNumber("Left Limelight average X", leftCalculatedX)),
                Commands.runOnce(() -> SmartDashboard.putNumber("Left Limelight average Y", leftCalculatedY)),

                Commands.runOnce(() -> SmartDashboard.putNumber("Right Limelight average X", rightCalculatedX)),
                Commands.runOnce(() -> SmartDashboard.putNumber("Right Limelight average Y", rightCalculatedY))

                    // Commands.race(
                    //     // new RunCommand(() -> driveRotateToTarget(pidAreaFinal, pidTXFinal, pidYawFinal), arm, elevator, claw, drivetrain).until(cameraStatusSupplier),
                    //     Commands.waitSeconds(2)
                    // ),
    
                    // Drop arm and elevator so the game piece can be intook
                    // Commands.race(
                    //     Commands.waitSeconds(5),
                    //     Commands.parallel( // End command once both arm and elevator have reached their target position
                    //         Commands.waitUntil(arm.atTargetPosition),
                    //         Commands.waitUntil(elevator.atTargetPosition),
                    //         Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup)),
                    //         Commands.sequence(
                    //             Commands.waitSeconds(0.25),
                    //             Commands.runOnce(() -> elevator.setTargetTicks(-160000))
                    //         )
                    //     )
                    // ),

                    // Open claw/Start claw intake rollers
                    // claw.setPower(-0.3),
                    // new WaitCommand(.5),
    
                    // // // Close claw/stop claw intake rollers/low background rolling to keep control of game piece
                    // claw.setPower(-0.15),
                    
                    // Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", false))
                
            );
            
        }
        else {
            return Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", false));
            //return runOnce(() -> SmartDashboard.putString("Limelight command status:", "Sequence cancelled"));
        }
    }

    public void initVisionPickupOnSubstation(OBJECT_TYPE objType) {
        initVisionCommands();
        //int armPositionTicks = ArmConstants.kArmStow;
            //int elevatorPositionTicks = ElevatorConstants.kElevatorStow;

            currentGameObject = objType;
            currentHeightPos = SCORE_POS.HIGH;
    
            //armPositionTicks = ArmConstants.kArmSubstation; // Pickup substation
            //elevatorPositionTicks = ElevatorConstants.kElevatorSubstation;

            goalYaw = 0; // Facing away from drivers, towards substation

            if (currentGameObject == OBJECT_TYPE.CONE) {
                goalArea = 21; // Goal area for cone substation pickup, area is an estimate because a different camera position was used, updated 2/23/2023
                limelightLeft.setPipeline(1);
                limelightRight.setPipeline(1);

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
                limelightLeft.setPipeline(2);
                limelightRight.setPipeline(2);
                

                PIDArea.setPID(SmartDashboard.getNumber("Ta P", 0), SmartDashboard.getNumber("Ta I", 0), SmartDashboard.getNumber("Ta D", 0));
                PIDTX.setPID(SmartDashboard.getNumber("Tx P", 0), SmartDashboard.getNumber("Tx I", 0), SmartDashboard.getNumber("Tx D", 0));
                PIDYaw.setPID(0, 0, 0);
            }

            //final int armPositionTicksFinal  = armPositionTicks;
            //final int elevatorPositionTicksFinal  = elevatorPositionTicks;
        }
    public CommandBase VisionPickupOnSubstation(OBJECT_TYPE objType) {
        if(limelightLeft != null) {

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
        goalYaw = 0; // All scoring is facing towards our drivers

        switch(currentGameObject) {
            case CONE:
                limelightLeft.setPipeline(3);
                limelightRight.setPipeline(3);
                limelightLeft.setLightState(LightMode.ON);
                limelightRight.setLightState(LightMode.ON);
                

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
                limelightLeft.setPipeline(4);
                limelightRight.setPipeline(4); // April tag pipeline
                
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

        if (limelightRight != null) {
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

        if ((limelightRight != null)&&(limelightLeft != null)) {
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

        if ((limelightLeft == null)&&(limelightLeft == null))
            return;

        ChassisSpeeds chassisSpeeds;

        boolean doBothHaveTarget = limelightRight.hasValidTarget()&&limelightLeft.hasValidTarget();
        SmartDashboard.putBoolean("Limelights have target", doBothHaveTarget);

        if(!doBothHaveTarget) {
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

            double calculatedX = getAvgArea((limelightLeft.getArea_avg() + limelightRight.getArea_avg()) / 2);
            double calculatedY = getAvgTX((limelightLeft.getXAngle_avg() + limelightRight.getXAngle_avg()) / 2);
            SmartDashboard.putNumber("Vision average X", calculatedX);
            SmartDashboard.putNumber("Vision average Y", calculatedY);

            if(limelightLeft.getPipeIndex()==4){
                if (NerdyMath.inRange(calculatedY, -2.2, 1) 
                    && calculatedX > 7) {
                chassisSpeeds = new ChassisSpeeds(0, 0, 0);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                currentCameraMode = CAMERA_MODE.ARRIVED; 
                limelightLeft.setLightState(LightMode.ON);
                limelightRight.setLightState(LightMode.ON);
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
                limelightLeft.setLightState(LightMode.ON);
                limelightRight.setLightState(LightMode.ON); 
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


    PIDController pidTX_driveToCubeOnGround = new PIDController(0.05, 0, 0.008);
    PIDController pidYaw_driveToCubeOnGround = new PIDController(0, 0, 0);
    PIDController pidArea_driveToCubeOnGround = new PIDController(0.8, 0.01, 0.02); //0.75 P OG

    public CommandBase driveToCubeOnGround(MotorClaw claw, int timeoutSec) // Commented out all movement
    {
        
        
        //PIDController pidArea, PIDController pidTX, PIDController pidYaw) {
        // Initialize all variables to 0
        double xSpeed = 0;
        double ySpeed = 0;
        double rotationSpeed = 0;

        if ((limelightRight == null)||(limelightLeft == null))
            return new InstantCommand(() -> SmartDashboard.putString("Status", "Limelight doesn't see anything"));

        // ChassisSpeeds chassisSpeeds;

        SmartDashboard.putBoolean("Right Vision has target", limelightRight.hasValidTarget());
        SmartDashboard.putBoolean("Left Vision has target", limelightLeft.hasValidTarget());


        double elapsedTime = timer.get();

        if(elapsedTime >= timeoutSec){
            // drivetrain.setModuleStates(SwerveDriveConstants.towModuleStates);
            // drivetrain.stopModules();
            limelightRight.setLightState(LightMode.BLINK);
            limelightLeft.setLightState(LightMode.BLINK);
            return new InstantCommand(() -> SmartDashboard.putString("Status", "Timeout"));
        }

        if(!limelightRight.hasValidTarget()&&!limelightLeft.hasValidTarget()) {
            // chassisSpeeds = new ChassisSpeeds(0, 0, 0);
            // SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            // drivetrain.setModuleStates(moduleStates);
            currentCameraMode = CAMERA_MODE.WAIT;
            return new InstantCommand(() -> SmartDashboard.putString("Status", "No valid Target"));
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

            double calculatedX = getAvgArea((limelightLeft.getArea_avg() + limelightRight.getArea_avg()) / 2);
            double calculatedY = getAvgTX((limelightLeft.getXAngle_avg() + limelightRight.getXAngle_avg()) / 2);
            SmartDashboard.putNumber("Vision average X", calculatedX);
            SmartDashboard.putNumber("Vision average Y", calculatedY);

            if(limelightRight.getPipeIndex()==2){ // TODO change it to cube-2
                if (NerdyMath.inRange(calculatedY, -15, 15) 
                    && NerdyMath.inRange(calculatedX, 4.0, 4.9)) { // OG 3.7 to 4.2 , 39 inches TODO: Da Vinci changed from 4.2, 5.0
                    // chassisSpeeds = new ChassisSpeeds(0, 0, 0);
                    // SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                    // drivetrain.setModuleStates(moduleStates);
                    currentCameraMode = CAMERA_MODE.ARRIVED;
                    limelightRight.setLightState(LightMode.ON);
                    limelightLeft.setLightState(LightMode.ON);
                return new InstantCommand(() -> SmartDashboard.putString("Status", "Limelight arrived"));
                }
            }
            xSpeed = pidArea_driveToCubeOnGround.calculate(calculatedX, goalArea);
            ySpeed = -pidTX_driveToCubeOnGround.calculate(calculatedY, goalTX);
            //rotationSpeed = pidYaw.calculate(drivetrain.getImu().getHeading(), goalYaw);
            
            if (NerdyMath.inRange(xSpeed, -.1, .1) &&
            NerdyMath.inRange(ySpeed, -.1, .1) &&
            NerdyMath.inRange(rotationSpeed, -.1, .1))
            {
                // chassisSpeeds = new ChassisSpeeds(0, 0, 0);
                // SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                // drivetrain.setModuleStates(moduleStates);
                currentCameraMode = CAMERA_MODE.ARRIVED;
                limelightRight.setLightState(LightMode.ON); 
                limelightLeft.setLightState(LightMode.ON); 
                return new InstantCommand(() -> SmartDashboard.putString("Status", "Limelight arrived"));

            }
            else{
                // chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
                // SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                // drivetrain.setModuleStates(moduleStates);
                currentCameraMode = CAMERA_MODE.ACTION;
            }
        }

        SmartDashboard.putString("Vision status", currentCameraMode.toString());
        SmartDashboard.putNumber("Vision X speed", xSpeed);
        SmartDashboard.putNumber("Vision Y speed", ySpeed);

        return new InstantCommand(() -> SmartDashboard.putString("Status", "FInished"));

    }

    PIDController pidTX_driveToGridTag = new PIDController(0.5, 0, 0.5);
    PIDController pidYaw_driveToGridTag = new PIDController(0, 0, 0);
    PIDController pidArea_driveToGridTag = new PIDController(0.5, 0.01, 1.5); //TODO: TUNE

    public void driveToGridTag(MotorClaw claw, int targetId) // if id == -1, don't care
    {
        
        //PIDController pidArea, PIDController pidTX, PIDController pidYaw) {
        // Initialize all variables to 0
        double xSpeed = 0;
        double ySpeed = 0;
        double rotationSpeed = 0;

        if ((limelightRight == null)&&(limelightLeft == null))
            return;

        ChassisSpeeds chassisSpeeds;        

        SmartDashboard.putBoolean("Vision has target", limelightRight.hasValidTarget());

        if(!limelightRight.hasValidTarget()&&!limelightLeft.hasValidTarget()) {
            chassisSpeeds = new ChassisSpeeds(0, 0, 0);
            SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrain.setModuleStates(moduleStates);
            currentCameraMode = CAMERA_MODE.WAIT;
        }
        else {
            int aid = limelightRight.getAprilTagID();
            if(targetId != -1) {
                if(targetId != aid) {
                    chassisSpeeds = new ChassisSpeeds(0, 0, 0);
                    SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                    drivetrain.setModuleStates(moduleStates);
                    currentCameraMode = CAMERA_MODE.WAIT;
                    return;
                }
            }

            double calculatedX = getAvgArea((limelightLeft.getArea() + limelightRight.getArea()) / 2);
            double calculatedY = getAvgTX((limelightLeft.getXAngle() + limelightRight.getXAngle()) / 2);
            SmartDashboard.putNumber("Vision average X", calculatedX);
            SmartDashboard.putNumber("Vision average Y", calculatedY);

            if(limelightRight.getPipeIndex()==4){// TAG ID? TODO
                if (NerdyMath.inRange(calculatedY, -6, 6) 
                    && NerdyMath.inRange(calculatedX, 3.5, 4.2)) { // TODO: TUNE FOR APRIL TAG
                    chassisSpeeds = new ChassisSpeeds(0, 0, 0);
                    SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                    drivetrain.setModuleStates(moduleStates);
                    currentCameraMode = CAMERA_MODE.ARRIVED; 
                    limelightRight.setLightState(LightMode.ON); 
                    limelightLeft.setLightState(LightMode.ON); 

                    return;
                }
            }
            xSpeed = pidArea_driveToGridTag.calculate(calculatedX, goalArea);
            ySpeed = -pidTX_driveToGridTag.calculate(calculatedY, goalTX);
            //rotationSpeed = pidYaw.calculate(drivetrain.getImu().getHeading(), goalYaw);
            
            if (NerdyMath.inRange(xSpeed, -.1, .1) &&
            NerdyMath.inRange(ySpeed, -.1, .1) &&
            NerdyMath.inRange(rotationSpeed, -.1, .1))
            {
                chassisSpeeds = new ChassisSpeeds(0, 0, 0);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                currentCameraMode = CAMERA_MODE.ARRIVED;
                limelightRight.setLightState(LightMode.ON); 
                limelightLeft.setLightState(LightMode.ON); 

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
                if((limelightLeft != null)&&(limelightRight != null)) {
                    SmartDashboard.putNumber("Vision Current Pipeline", limelightLeft.getPipeIndex());
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
                    if((limelightLeft != null)&&(limelightRight != null))  {
                        return limelightLeft.hasValidTarget();
                    }
                    return false;
                });
                tab.addNumber("Pipeline", () -> {
                    if((limelightLeft != null)&&(limelightRight != null))  {
                        return limelightLeft.getPipeIndex();
                    }
                    return -1;
                });
                tab.addString("Current Limelight", () -> {
                    if((limelightLeft != null)&&(limelightRight != null))  {
                        return limelightLeft.getName();
                    }
                    return "L + ratio";
                });
            case MINIMAL:
                break;
        }
    }
}
