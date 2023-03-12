// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Imu;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.SwerveAutos;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.TheGreatBalancingAct;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.SwerveAutos.StartPosition;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.SwerveModuleType;
import frc.robot.subsystems.vision.VROOOOM.SCORE_POS;
import frc.robot.commands.SwerveJoystickCommand.DodgeDirection;
import frc.robot.util.BadPS4;
import frc.robot.util.CommandBadPS4;


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
  public static Imu imu = new Imu();
  public static SwerveDrivetrain swerveDrive;

  private final CommandBadPS4 driverController = new CommandBadPS4(
      ControllerConstants.kDriverControllerPort);
  private final BadPS4 badPS5 = driverController.getHID();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandBadPS4 operatorController = new CommandBadPS4(
      ControllerConstants.kOperatorControllerPort);
  private final BadPS4 badPS4 = operatorController.getHID();
  // private final Joystick joystick = new Joystick(2);

  private final LOG_LEVEL loggingLevel = LOG_LEVEL.MINIMAL;

  private final POVButton upButton = new POVButton(badPS4, 0);
  private final POVButton rightButton = new POVButton(badPS4, 90);
  private final POVButton downButton = new POVButton(badPS4, 180);
  private final POVButton leftButton = new POVButton(badPS4, 270);

  private final POVButton upButtonDriver = new POVButton(badPS5, 0);
  private final POVButton rightButtonDriver = new POVButton(badPS5, 90);
  private final POVButton downButtonDriver = new POVButton(badPS5, 180);
  private final POVButton leftButtonDriver = new POVButton(badPS5, 270);

  private SendableChooser<Supplier<CommandBase>> autoChooser = new SendableChooser<Supplier<CommandBase>>();
  private SendableChooser<StartPosition> positionChooser = new SendableChooser<StartPosition>();
  private SendableChooser<SCORE_POS> scoreChooser = new SendableChooser<SCORE_POS>();
  private SendableChooser<Alliance> allianceChooser = new SendableChooser<Alliance>();

  private SCORE_POS scorePos = SCORE_POS.MID;
  private StartPosition startPos = StartPosition.RIGHT;
  private Alliance alliance = Alliance.Invalid;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //if (IsSwerveDrive) {
      try {
        swerveDrive = new SwerveDrivetrain(imu, SwerveModuleType.MAG_ENCODER);
      } catch (IllegalArgumentException e) {
        DriverStation.reportError("Illegal Swerve Drive Module Type", e.getStackTrace());
      }

      initAutoChoosers();
      
    configureBindings();
    
  }

  public void initDefaultCommands() {
      swerveDrive.setDefaultCommand(
        new SwerveJoystickCommand(
          swerveDrive,
          // Translation Y
          // () -> -joystick.getY(),
          () -> -driverController.getLeftY(),

          // Translation X
          driverController::getLeftX,
          // joystick::getX,

          // Rotation
          // joystick::getTwist,
          // () -> 0.0,
          driverController::getRightX,
          // () -> true,

          // Field oriented
          badPS5::getSquareButton,
          badPS5::getL2Button,
          // driverControllerButtons::getTriangleButton,
          // Dodge
          badPS5::getR3Button,
          // Dodging
          () -> {
            // if (badPS4.getL2Button()) {
            //   return DodgeDirection.LEFT;
            // } 
            // if (badPS4.getR2Button()) {
            //   return DodgeDirection.RIGHT;
            // }
            return DodgeDirection.NONE;
          },
          // Precision/"Sniper Button"
          badPS5::getR2Button
        ));
  }

  private void configureBindings() {
      driverController.share().onTrue(new InstantCommand(imu::zeroHeading));
      driverController.options().onTrue(new InstantCommand(swerveDrive::resetEncoders));

      driverController.R1().whileTrue(new TurnToAngle(180, swerveDrive));
      driverController.L1().whileTrue(new TurnToAngle(0, swerveDrive));
      
      driverController.triangle().whileTrue(new TheGreatBalancingAct(swerveDrive));

      // driverController.L2().whileTrue(new Dodge(swerveDrive, -driverController.getLeftY(), driverController.getLeftX(), true));
      // driverController.R2().whileTrue(new Dodge(swerveDrive, -driverController.getLeftY(), driverController.getLeftX(), false));

      // ====== Vision Bindings ====== 
      // driverController.L2().whileTrue(vision.VisionPickupOnSubstation(OBJECT_TYPE.CONE))
      //   .onFalse(Commands.runOnce(swerveDrive::stopModules, swerveDrive));
      // driverController.R2().whileTrue(vision.VisionPickupOnSubstation(OBJECT_TYPE.CUBE))
      //   .onFalse(Commands.runOnce(swerveDrive::stopModules, swerveDrive));
  }

  private void initAutoChoosers() {
    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");
    SmartDashboard.putBoolean("Dummy Auto", false);

    autoChooser.addOption("Charge only", () -> SwerveAutos.chargeAuto(swerveDrive, startPos, alliance, 0, false));
    autoChooser.addOption("Backward Auto", () -> SwerveAutos.driveBackwardAuto(swerveDrive));
    autoChooser.setDefaultOption("Old Charge", () -> SwerveAutos.backupChargeAuto(swerveDrive));
    autoChooser.addOption("Test Auto",  () -> Commands.runOnce(() -> SmartDashboard.putBoolean("Dummy Auto", true)));
    // autoChooser.addOption("Old One Piece", () -> SwerveAutos.backupTwoPieceChargeAuto(swerveDrive, arm, elevator, motorClaw));
    autosTab.add("Selected Auto", autoChooser);
    
    positionChooser.setDefaultOption("Right", StartPosition.RIGHT);
    positionChooser.addOption("Left", StartPosition.LEFT);
    positionChooser.addOption("Middle", StartPosition.MIDDLE);
    positionChooser.addOption("Right", StartPosition.RIGHT);
    autosTab.add("Start Position", positionChooser);
    autosTab.addString("Selected Start Position", () -> startPos.toString());

    // TODO: Implement changing score position in the autos
    scoreChooser.setDefaultOption("Mid", SCORE_POS.MID);
    scoreChooser.addOption("Hybrid", SCORE_POS.LOW);
    scoreChooser.addOption("Mid", SCORE_POS.MID);
    scoreChooser.addOption("High", SCORE_POS.HIGH);
    autosTab.add("Score Position", scoreChooser);

    allianceChooser.setDefaultOption("Red", Alliance.Red);
    allianceChooser.addOption("Red", Alliance.Red);
    allianceChooser.addOption("Blue", Alliance.Blue);
    autosTab.add("Alliance", allianceChooser);

    autosTab.addString("Selected Score Position", () -> scorePos.toString());
  }
  
  public void initShuffleboard() {
    imu.initShuffleboard(loggingLevel);
    swerveDrive.initShuffleboard(loggingLevel);
    swerveDrive.initModuleShuffleboard(loggingLevel);
  }

  public void reportAllToSmartDashboard() {
    imu.reportToSmartDashboard(loggingLevel);
    swerveDrive.reportToSmartDashboard(loggingLevel);
    swerveDrive.reportModulesToSmartDashboard(loggingLevel);
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    startPos = positionChooser.getSelected();
    scorePos = scoreChooser.getSelected();
    alliance = allianceChooser.getSelected();
    Command currentAuto = autoChooser.getSelected().get();
    return currentAuto;
  }

  public void autonomousInit() {
    imu.zeroHeading();
  }
}
