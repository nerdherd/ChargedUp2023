// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import frc.robot.subsystems.claw.MotorClaw;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.ChargeAutos;
import frc.robot.commands.PathPlannerAutos;
import frc.robot.commands.SwerveAutos;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.TestAutos;
import frc.robot.commands.TheGreatBalancingAct;
import frc.robot.commands.VisionAllLowAuto;
import frc.robot.commands.VisionCableSideAuto;
import frc.robot.commands.SwerveAutos.StartPosition;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.imu.NavX;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;
import frc.robot.subsystems.swerve.SwerveDrivetrain.SwerveModuleType;
import frc.robot.subsystems.vision.VROOOOM;
import frc.robot.subsystems.vision.VROOOOM.OBJECT_TYPE;
import frc.robot.subsystems.vision.VROOOOM.SCORE_POS;
import frc.robot.commands.SwerveJoystickCommand.DodgeDirection;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */// 10.6.87.98:5800
public class RobotContainer {

  public Arm arm = new Arm();
  public Elevator elevator = new Elevator();
  public MotorClaw motorClaw = new MotorClaw();
  public Gyro imu = new NavX();
  // public Gyro imu = new Pigeon(60);
  public SwerveDrivetrain swerveDrive;
  public VROOOOM vision;

  private final CommandPS4Controller driverController = new CommandPS4Controller(
      ControllerConstants.kDriverControllerPort);
  private final PS4Controller badPS5 = driverController.getHID();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller operatorController = new CommandPS4Controller(
      ControllerConstants.kOperatorControllerPort);
  private final PS4Controller badPS4 = operatorController.getHID();
  // private final Joystick joystick = new Joystick(2);

  private final LOG_LEVEL loggingLevel = LOG_LEVEL.ALL;

  private final POVButton upButton = new POVButton(badPS4, 0);
  private final POVButton rightButton = new POVButton(badPS4, 90);
  private final POVButton downButton = new POVButton(badPS4, 180);
  private final POVButton leftButton = new POVButton(badPS4, 270);

  private final POVButton upButtonDriver = new POVButton(badPS5, 0);
  private final POVButton rightButtonDriver = new POVButton(badPS5, 90);
  private final POVButton downButtonDriver = new POVButton(badPS5, 180);
  private final POVButton leftButtonDriver = new POVButton(badPS5, 270);

  private SendableChooser<Supplier<CommandBase>> autoChooser = new SendableChooser<Supplier<CommandBase>>();
  // private SendableChooser<StartPosition> positionChooser = new SendableChooser<StartPosition>();
  // private SendableChooser<SCORE_POS> scoreChooser = new SendableChooser<SCORE_POS>();
  private SendableChooser<Alliance> allianceChooser = new SendableChooser<Alliance>();

  private SCORE_POS scorePos = SCORE_POS.MID;
  private StartPosition startPos = StartPosition.RIGHT;
  private Alliance alliance = Alliance.Invalid;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    try {
      swerveDrive = new SwerveDrivetrain(imu, SwerveModuleType.CANCODER);
      vision = new VROOOOM(arm, elevator, motorClaw, swerveDrive);
    } catch (IllegalArgumentException e) {
      DriverStation.reportError("Illegal Swerve Drive Module Type", e.getStackTrace());
    }

    // Initialize vision after swerve has been initialized
    vision = new VROOOOM(arm, elevator, motorClaw, swerveDrive);

    // this.alliance = DriverStation.getAlliance();
    initAutoChoosers();

    // Configure the trigger bindings
    configureBindings();

    DriverStation.reportWarning("Initalization complete", false);
  }

  public void initDefaultCommands() {
    arm.init();
    arm.initTargetTicks();
    arm.setDefaultCommand(
      new RunCommand(
        () -> {
          arm.moveArmMotionMagicJoystick(operatorController.getLeftY(), elevator.percentExtended());
          // SmartDashboard.putNumber("Arm input", operatorController.getLeftY());
        }, 
        arm
      ));
    
    elevator.setDefaultCommand(
      new RunCommand(
        () -> {
          elevator.moveElevatorJoystick(operatorController.getRightY() * -0.125, arm.getArmAngle());
          // SmartDashboard.putNumber("Elevator input", operatorController.getRightY());
        }, 
        elevator
      ));

    swerveDrive.setDefaultCommand(
      new SwerveJoystickCommand(
        swerveDrive,
        () -> -driverController.getLeftY(), // Horizontal translation
        driverController::getLeftX, // Vertical Translation
        // () -> 0.0, // debug
        driverController::getRightX, // Rotation
        badPS5::getSquareButton, // Field oriented
        badPS5::getL2Button, // Towing
        // Dodge
        // () -> {return badPS5.getL1Button() || badPS5.getR1Button();},
        () -> false,
        // Dodging
        () -> {
          // if (badPS5.getL1Button()) {
          //   return DodgeDirection.LEFT;
          // } 
          // if (badPS5.getR1Button()) {
          //   return DodgeDirection.RIGHT;
          // }
          return DodgeDirection.NONE;
        },
        badPS5::getR2Button, // Precision/"Sniper Button"
        () -> badPS5.getR1Button() || badPS5.getL1Button(), // Turn to angle
        () -> { // Turn To angle Direction
          if (badPS5.getR1Button()) {
            return 180.0;
          } else {
            return 0.0;
          }
        }
      ));
  }

  private void configureBindings() {
    // Note: whileTrue() does not restart the command if it ends while the button is
    // still being held
    // These button bindings are chosen for testing, and may be changed based on
    // driver preference
    
    upButton.whileTrue(new InstantCommand(() -> arm.moveArmMotionMagicButton(ArmConstants.kArmStow)))
      .onFalse(Commands.runOnce(arm::stopArm));
    leftButton.whileTrue(new InstantCommand(() -> arm.moveArmMotionMagicButton(ArmConstants.kArmScore)))
      .onFalse(Commands.runOnce(arm::stopArm));
    rightButton.whileTrue(new InstantCommand(() -> arm.moveArmMotionMagicButton(ArmConstants.kArmSubstation)))
      .onFalse(Commands.runOnce(arm::stopArm));
    downButton.whileTrue(new InstantCommand(() -> arm.moveArmMotionMagicButton(ArmConstants.kArmGroundPickup)))
      .onFalse(Commands.runOnce(arm::stopArm));
    
    operatorController.triangle().whileTrue(elevator.moveElevatorHigh(arm::getArmAngle))
      .onFalse(Commands.runOnce(elevator::setPowerZero));
    operatorController.square().whileTrue(elevator.moveElevatorMid(arm::getArmAngle))
      .onFalse(Commands.runOnce(elevator::setPowerZero));
    operatorController.circle().whileTrue(elevator.moveElevator(ElevatorConstants.kElevatorSubstation, arm::getArmAngle))
      .onFalse(Commands.runOnce(elevator::setPowerZero));
    operatorController.cross().whileTrue(elevator.moveElevatorStow(arm::getArmAngle))
      .onFalse(Commands.runOnce(elevator::setPowerZero));
  
    operatorController.share().onTrue(Commands.runOnce(arm::resetEncoderStow));
    operatorController.options().onTrue(Commands.runOnce(elevator::resetEncoder));
    
    operatorController.L1().whileTrue(motorClaw.setPower(0.8))
        .onFalse(motorClaw.setPowerZero());
    operatorController.R1().whileTrue(motorClaw.setPower(-.3))
        .onFalse(motorClaw.setPower(-0.07));

    driverController.share().onTrue(new InstantCommand(imu::zeroHeading));
    driverController.options().onTrue(new InstantCommand(swerveDrive::resetEncoders));

    // driverController.PS().onTrue(new InstantCommand(() -> swerveDrive.resetOdometry(new Pose2d())));
    // driverController.R1().whileTrue(new TurnToAngle(180, swerveDrive)); // Replaced with turn to angles in the drive command
    // driverController.L1().whileTrue(new TurnToAngle(0, swerveDrive));
    
    driverController.triangle().whileTrue(new TheGreatBalancingAct(swerveDrive));
    driverController.circle()
      .whileTrue(Commands.run(() -> swerveDrive.setVelocityControl(true)))
      .whileFalse(Commands.run(() -> swerveDrive.setVelocityControl(false)));

    // driverController.L2().whileTrue(new Dodge(swerveDrive, -driverController.getLeftY(), driverController.getLeftX(), true));
    // driverController.R2().whileTrue(new Dodge(swerveDrive, -driverController.getLeftY(), driverController.getLeftX(), false));

    // ====== Vision Bindings ====== 
    driverController.cross().whileTrue(vision.VisionPickupOnSubstation(OBJECT_TYPE.CONE))
      .onFalse(Commands.runOnce(swerveDrive::stopModules, swerveDrive));
    // driverController.R2().whileTrue(vision.VisionPickupOnSubstation(OBJECT_TYPE.CUBE))
    //   .onFalse(Commands.runOnce(swerveDrive::stopModules, swerveDrive));

    operatorController.L2().whileTrue(vision.VisionPickupOnGround(OBJECT_TYPE.CUBE));
    operatorController.R2().whileTrue(vision.VisionScore(OBJECT_TYPE.CUBE, SCORE_POS.HIGH));

    //operatorController.L2().onTrue(vision.updateCurrentGameObjects(OBJECT_TYPE.CONE));
    //operatorController.R2().onTrue(vision.updateCurrentGameObject(OBJECT_TYPE.CUBE));

    //upButtonDriver.onTrue(vision.updateCurrentHeight(SCORE_POS.HIGH));
    //rightButtonDriver.onTrue(vision.updateCurrentHeight(SCORE_POS.MID));
    //downButtonDriver.onTrue(vision.updateCurrentHeight(SCORE_POS.LOW));

    // upButtonDriver.whileTrue(vision.VisionScore(OBJECT_TYPE.CONE, SCORE_POS.HIGH))
    // .onFalse(Commands.runOnce(swerveDrive::stopModules, swerveDrive));
    // leftButtonDriver.whileTrue(vision.VisionScore(OBJECT_TYPE.CONE, SCORE_POS.MID))
    //   .onFalse(Commands.runOnce(swerveDrive::stopModules, swerveDrive));

    
    // rightButtonDriver.whileTrue(vision.VisionScore(OBJECT_TYPE.CUBE, SCORE_POS.HIGH))
    //   .onFalse(Commands.runOnce(swerveDrive::stopModules, swerveDrive));
    // downButtonDriver.whileTrue(vision.VisionScore(OBJECT_TYPE.CUBE, SCORE_POS.MID))
    // .onFalse(Commands.runOnce(swerveDrive::stopModules, swerveDrive));
  }

  private void initAutoChoosers() {
    // Remember to load the pathplanner paths here
    final String[] paths = {
      "TestPath", "ChargeAroundLEFT", "TaxiRIGHT", "TaxiLEFT", "TestSquare", "Test Line", "TestSquare3"
    };

    for (String path : paths) {
      PathPlannerAutos.initPath(path);
    }
    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");

    // New PathPlanner Autos

    autoChooser.addOption("Do Nothing", Commands::none);
    autoChooser.addOption("Path Planner Test Auto", () -> PathPlannerAutos.pathplannerAuto("TestPath", swerveDrive));
    autoChooser.addOption("Path Planner Charge Around LEFT", () -> PathPlannerAutos.pathplannerAuto("ChargeAroundLEFT", swerveDrive));
    autoChooser.addOption("Path Planner TaxiRIGHT", () -> PathPlannerAutos.pathplannerAuto("TaxiRIGHT", swerveDrive));
    autoChooser.addOption("Path Planner TaxiLEFT", () -> PathPlannerAutos.pathplannerAuto("TaxiLEFT", swerveDrive));
    autoChooser.addOption("Path Planner TestSquare", () -> PathPlannerAutos.pathplannerAuto("TestSquare", swerveDrive));
    autoChooser.addOption("Path Planner Test3", () -> PathPlannerAutos.pathplannerAuto("Test Line", swerveDrive));
    autoChooser.addOption("Path Planner TestSquare3", () -> PathPlannerAutos.pathplannerAuto("TestSquare3", swerveDrive));

    // TODO: Flip y-values in all old autos (pre-August 2023)

    // No alliance parameter.
    autoChooser.addOption("Preload Charge", () -> ChargeAutos.preloadHighChargeMiddle(swerveDrive, arm, elevator, motorClaw)); // Same as preload charge auto
    autoChooser.addOption("Preload Slow taxi charge", () -> TestAutos.preloadTaxiChargeBackwardsSLOW(swerveDrive, motorClaw, arm, elevator));
    autoChooser.addOption("Preload Slow charge", () -> TestAutos.preloadChargeBackwardsSLOW(swerveDrive, motorClaw, arm, elevator));
    // autoChooser.addOption("Slow charge", () -> TestAutos.chargeBackwardsSLOW(swerveDrive, motorClaw, arm, elevator));
    // autoChooser.addOption("Slow taxi charge", () -> TestAutos.taxiChargeBackwardsSLOW(swerveDrive, motorClaw, arm, elevator));
    
    // autoChooser.addOption("Charge", () -> ChargeAutos.chargeMiddle(swerveDrive)); // Same as preload charge auto
    
    // autoChooser.addOption("Custom Preload High Center Charge Taxi", () -> ChargeAutos.customPreloadHighChargeTaxiMiddle(swerveDrive, arm, elevator, motorClaw));
    // autoChooser.addOption("Gyro Charge", () -> ChargeAutos.gyroCharge(swerveDrive, true));

    // Has and uses alliance parameter.
    // autoChooser.addOption("Vision Two Piece Smooth (Alliance)", () -> VisionAutos.zoomTwoPieceAuto(swerveDrive, vision, arm, elevator, motorClaw, alliance));
    // autoChooser.addOption("Vision Two Piece Cable (Alliance)", () -> VisionAutos.cableZoomTwoPieceAuto(swerveDrive, vision, arm, elevator, motorClaw, alliance));
    autoChooser.addOption("Smooth Low Cube Auto", () -> VisionAllLowAuto.ThreeCubesAutoFast(swerveDrive, vision, arm, elevator, motorClaw, alliance));
    autoChooser.addOption("Cable Low Cube Auto", () -> VisionCableSideAuto.LowAuto(swerveDrive, vision, arm, elevator, motorClaw, alliance));
    autoChooser.addOption("Cable High Cube Auto", () -> VisionCableSideAuto.HighAuto(swerveDrive, vision, arm, elevator, motorClaw, alliance));


    // Have alliance parameter but do not use it.
    autoChooser.addOption("LAR Auto", () -> SwerveAutos.preloadChargeAuto(swerveDrive, arm, elevator, motorClaw, StartPosition.MIDDLE, SCORE_POS.HIGH, 0, false, alliance));
    autoChooser.addOption("Preload Taxi Auto", () -> SwerveAutos.preloadBackwardAuto(swerveDrive, arm, elevator, motorClaw, SCORE_POS.HIGH, alliance));

    allianceChooser.setDefaultOption("Red", Alliance.Red);
    allianceChooser.addOption("Red", Alliance.Red);
    allianceChooser.addOption("Blue", Alliance.Blue);
    autosTab.add("Alliance", allianceChooser);

    autosTab.addString("Selected Score Position", () -> scorePos.toString());
  }
  
  public void initShuffleboard() {
    ShuffleboardTab joystickTab = Shuffleboard.getTab("OI");
    joystickTab.addNumber("Driver LX", driverController::getLeftX);
    joystickTab.addNumber("Driver LY", driverController::getLeftY);
    joystickTab.addNumber("Driver RX", driverController::getRightX);
    joystickTab.addNumber("Driver RY", driverController::getRightY);

    imu.initShuffleboard(loggingLevel);
    arm.initShuffleboard(loggingLevel);
    elevator.initShuffleboard(loggingLevel);
    motorClaw.initShuffleboard(loggingLevel);
    swerveDrive.initShuffleboard(loggingLevel);
    swerveDrive.initModuleShuffleboard(loggingLevel);
    vision.initShuffleboard(loggingLevel);
  }

  public void reportAllToSmartDashboard() {
    // SmartDashboard.putNumber("Elevator FF", Math.sin(arm.getArmAngle()) * ElevatorConstants.kArbitraryFF);
    // SmartDashboard.putNumber("Arm FF", -(ArmConstants.kStowedFF + ArmConstants.kDiffFF * elevator.percentExtended()) * Math.cos(arm.getArmAngle()));
    imu.reportToSmartDashboard(loggingLevel);
    motorClaw.reportToSmartDashboard(loggingLevel);
    arm.reportToSmartDashboard(loggingLevel);
    elevator.reportToSmartDashboard(loggingLevel);
    vision.reportToSmartDashboard(loggingLevel);
    swerveDrive.reportToSmartDashboard(loggingLevel);
    swerveDrive.reportModulesToSmartDashboard(loggingLevel);
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // startPos = positionChooser.getSelected();
    // scorePos = scoreChooser.getSelected();
    alliance = allianceChooser.getSelected();
    // Command currentAuto = VisionAllLowAuto.ThreeCubesAuto(swerveDrive, vision, arm, elevator, motorClaw, Alliance.Red);
    Command currentAuto = autoChooser.getSelected().get();

    swerveDrive.setDriveMode(DRIVE_MODE.AUTONOMOUS);
    return currentAuto;
  }
}
