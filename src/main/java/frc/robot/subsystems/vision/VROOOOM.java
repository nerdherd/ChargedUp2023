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
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.NerdyMath;

import edu.wpi.first.wpilibj2.command.Commands;

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

    public CommandBase VisionPickup() {
        if(currentLimelight != null) {
            int armPositionTicks = ArmConstants.kArmStow;
            int elevatorPositionTicks = ElevatorConstants.kElevatorStow;
    
            rotationIsNeeded = false; // Reset rotation variable
    
            switch(currentHeightPos) {
                case HIGH:
                    armPositionTicks = ArmConstants.kArmSubstation; // Pickup substation
                    elevatorPositionTicks = ElevatorConstants.kElevatorSubstation;
                    currentLimelight = limelightHigh;
                    rotationIsNeeded = true;
                    goalYaw = 0; // Facing away from drivers, towards substation
    
                    if (currentGameObject == OBJECT_TYPE.CONE) {
                        goalArea = 21; // Goal area for cone substation pickup, area is an estimate because a different camera position was used, updated 2/23/2023
                    } else {
                        goalArea = 0; // Goal area for cube substation pickup
                    }
                    break;
                    
                case MID:
                    return Commands.runOnce(() -> SmartDashboard.putString("Vision Message", "you cant pickup mid"));

                case LOW:
                    armPositionTicks = ArmConstants.kArmGroundPickup; // Ground pickup
                    elevatorPositionTicks = ElevatorConstants.kElevatorStow;
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
                    PIDArea = new PIDController(SmartDashboard.getNumber("Ta P", 0), SmartDashboard.getNumber("Ta I", 0), SmartDashboard.getNumber("Ta D", 0)); // NOT SURE IF CORRECT, Updated 2/23/2023
                    PIDTX = new PIDController(SmartDashboard.getNumber("Tx P", 0), SmartDashboard.getNumber("Tx I", 0), SmartDashboard.getNumber("Tx D", 0));
                    PIDYaw = new PIDController(SmartDashboard.getNumber("Yaw P", 0), SmartDashboard.getNumber("Yaw I", 0), SmartDashboard.getNumber("Yaw D", 0));
                    break;
    
                case CUBE:
                    currentLimelight.setPipeline(2);
                    PIDArea = new PIDController(SmartDashboard.getNumber("Ta P", 0), SmartDashboard.getNumber("Ta I", 0), SmartDashboard.getNumber("Ta D", 0)); // NOT SURE IF CORRECT, Updated 2/23/2023
                    PIDTX = new PIDController(SmartDashboard.getNumber("Tx P", 0), SmartDashboard.getNumber("Tx I", 0), SmartDashboard.getNumber("Tx D", 0));
                    PIDYaw = new PIDController(SmartDashboard.getNumber("Yaw P", 0), SmartDashboard.getNumber("Yaw I", 0), SmartDashboard.getNumber("Yaw D", 0));
                    break;
            }
    
            final int armPositionTicksFinal = armPositionTicks;
            final int elevatorPositionTicksFinal = elevatorPositionTicks;
    
            RunCommand currentVisionRunCommand;
    
            if (rotationIsNeeded) {
                currentVisionRunCommand = new RunCommand(() -> driveRotateToTarget(PIDArea, PIDTX, PIDYaw));
            } else {
                currentVisionRunCommand = new RunCommand(() -> skrttttToTarget(PIDArea, PIDTX));
            }
            
            return Commands.parallel(
                // Constantly run elevator and arm motion magic
                elevator.moveElevator(arm::getArmAngle),
                arm.moveArm(elevator::percentExtended),

                Commands.sequence(
                    Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", true)),
                    Commands.runOnce(() -> initVisionCommands()),
    
                    // Arm and elevator to selected position
                    Commands.race(
                        Commands.waitSeconds(5),
                        Commands.parallel( // End command once both arm and elevator have reached their target position
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.runOnce(() -> arm.setTargetTicks(armPositionTicksFinal)),
                            Commands.runOnce(() -> elevator.setTargetTicks(elevatorPositionTicksFinal))
                        )
                    ),
                    
                    currentVisionRunCommand.until(cameraStatusSupplier).withTimeout(5), // Timeout after 30 seconds
    
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
            return Commands.runOnce(() -> SmartDashboard.putString("Limelight command status:", "Commands.sequence cancelled"));
        }
    }

    public CommandBase VisionScore() {
        if(currentLimelight != null) {
            int armPositionTicks = ArmConstants.kArmStow;
            int elevatorPositionTicks = ElevatorConstants.kElevatorStow;
    
            rotationIsNeeded = true;
            goalYaw = 180; // All scoring is facing towards our drivers
    
            switch(currentGameObject) {
                case CONE:
                    currentLimelight = limelightHigh;
                    currentLimelight.setPipeline(3); // Tape pipeline
                    PIDArea = new PIDController(SmartDashboard.getNumber("Ta P", 0), SmartDashboard.getNumber("Ta I", 0), SmartDashboard.getNumber("Ta D", 0)); // NOT SURE IF CORRECT, Updated 2/23/2023
                    PIDTX = new PIDController(SmartDashboard.getNumber("Tx P", 0), SmartDashboard.getNumber("Tx I", 0), SmartDashboard.getNumber("Tx D", 0));
                    PIDYaw = new PIDController(SmartDashboard.getNumber("Yaw P", 0), SmartDashboard.getNumber("Yaw I", 0), SmartDashboard.getNumber("Yaw D", 0));
                    goalArea = 0.15; // Unsure if correct, updated 2/23/2023
                    break;
    
                case CUBE:
                    currentLimelight = limelightLow;
                    currentLimelight.setPipeline(4); // April tag pipeline
                    PIDArea = new PIDController(SmartDashboard.getNumber("Ta P", 0), SmartDashboard.getNumber("Ta I", 0), SmartDashboard.getNumber("Ta D", 0)); // NOT SURE IF CORRECT, Updated 2/23/2023
                    PIDTX = new PIDController(SmartDashboard.getNumber("Tx P", 0), SmartDashboard.getNumber("Tx I", 0), SmartDashboard.getNumber("Tx D", 0));
                    PIDYaw = new PIDController(SmartDashboard.getNumber("Yaw P", 0), SmartDashboard.getNumber("Yaw I", 0), SmartDashboard.getNumber("Yaw D", 0));
                    goalArea = 2; // April tag target area, unsure if correct, updated 2/23/2023
                    break;
            }
    
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
    
            final int armPositionTicksFinal = armPositionTicks;
            final int elevatorPositionTicksFinal = elevatorPositionTicks;
    
            RunCommand currentVisionRunCommand;
    
            if (rotationIsNeeded) {
                currentVisionRunCommand = new RunCommand(() -> driveRotateToTarget(PIDArea, PIDTX, PIDYaw));
            } else {
                currentVisionRunCommand = new RunCommand(() -> skrttttToTarget(PIDArea, PIDTX));
            }
    
            return Commands.parallel(
                // Constantly run elevator and arm motion magic
                elevator.moveElevator(arm::getArmAngle),
                arm.moveArm(elevator::percentExtended),
                
                Commands.sequence(
                    Commands.parallel(
                        Commands.runOnce(() -> SmartDashboard.putString("Vision Score Stage", "Stow")),
                        Commands.runOnce(() -> initVisionCommands())
                    ),
                    
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
                    
                    currentVisionRunCommand.until(cameraStatusSupplier).withTimeout(5),
    
                    // Arm and elevator to selected position
                    Commands.race(
                        Commands.waitSeconds(5),
                        Commands.parallel( // End command once both arm and elevator have reached their target position
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.runOnce(() -> arm.setTargetTicks(armPositionTicksFinal)),
                            Commands.runOnce(() -> elevator.setTargetTicks(elevatorPositionTicksFinal))
                        )
                    ),
                    
                    // Open claw/eject piece with rollers
                    claw.setPower(1),
                    // Wait 1 second
                    Commands.waitSeconds(1),
    
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
                    
                    Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Score Running", false))
                )
            );
        }
        else {
            return Commands.runOnce(() -> SmartDashboard.putString("Limelight command status:", "Commands.sequence cancelled"));
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
        return Commands.runOnce(
            () -> {
                currentGameObject = oType;
            });
    }

    public CommandBase updateCurrentHeight(SCORE_POS sPos) {
        return Commands.runOnce(
            () -> {
                currentHeightPos = sPos;
            });
    }

    public void setLimelightHigh(){
        if (limelightHigh == null){
            SmartDashboard.putString("Limelight Not Found", "Limelight High Not Found");
            return;
        }      
        currentLimelight = limelightHigh;
    }
    public void setLimelightLow(){
        if (limelightLow == null){
            SmartDashboard.putString("Limelight Not Found", "Limelight Low Not Found");
            return;
        }      
        currentLimelight = limelightLow;
    }
    public void setLimelightPipeline(int pipeline){
        if(currentLimelight == null){
            SmartDashboard.putString("Limelight Not Found", "Current Limelight Is Null");
            return;
        }
        currentLimelight.setPipeline(pipeline);
    }

    public void setGoalArea(double goalArea){
        this.goalArea = goalArea;
    }
    public void setGoalTX(double goalTX){
        this.goalTX = goalTX;
    }
    public void setGoalYaw(double goalYaw){
        this.goalYaw = goalYaw;
    }

    // Smartdashboard

    @Override
    public void reportToSmartDashboard() {
        SmartDashboard.putString("Vision Current Object", currentGameObject.toString());
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

        tab.addString("Vision Current Object", () -> currentGameObject.toString());
        tab.addString("Vision Current Height", () -> currentHeightPos.toString());
        if(currentLimelight != null) {
            tab.addString("Vision Current Limelight", () -> currentLimelight.getName());
            tab.addNumber("Vision Pipeline", () -> currentLimelight.getPipeIndex());
        }
        else {
            tab.addString("Vision Current Limelight", () -> "L + ratio");
        }
    }
}
