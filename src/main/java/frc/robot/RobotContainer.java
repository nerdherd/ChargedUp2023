// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.PathPlannerAutos;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.imu.NavX;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;
import frc.robot.subsystems.swerve.SwerveDrivetrain.SwerveModuleType;
import frc.robot.commands.SwerveJoystickCommand.DodgeDirection;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public Gyro imu = new NavX();
  // public Gyro imu = new Pigeon(60);
  public SwerveDrivetrain swerveDrive;

  private final CommandPS4Controller driverController = new CommandPS4Controller(
      ControllerConstants.kDriverControllerPort);
  private final PS4Controller badPS5 = driverController.getHID();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller operatorController = new CommandPS4Controller(
      ControllerConstants.kOperatorControllerPort);
  private final PS4Controller badPS4 = operatorController.getHID();
  // private final Joystick joystick = new Joystick(2);

  private final LOG_LEVEL loggingLevel = LOG_LEVEL.ALL;

  private SendableChooser<Supplier<CommandBase>> autoChooser = new SendableChooser<Supplier<CommandBase>>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    try {
      swerveDrive = new SwerveDrivetrain(imu, SwerveModuleType.CANCODER);
    } catch (IllegalArgumentException e) {
      DriverStation.reportError("Illegal Swerve Drive Module Type", e.getStackTrace());
    }

    initAutoChoosers();

    // Configure the trigger bindings
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

        // Towing
        badPS5::getL2Button,

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
        // Precision/"Sniper Button"
        badPS5::getR2Button,
        // Turn to angle
        // () -> false,
        () -> badPS5.getR1Button() || badPS5.getL1Button(),
        // Turn To angle Direction
        () -> {
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
    driverController.share().onTrue(Commands.runOnce(imu::zeroHeading));
    driverController.options().onTrue(Commands.runOnce(swerveDrive::resetEncoders));
  }

  private void initAutoChoosers() {
    autoChooser.addOption("Do Nothing", Commands::none);
    autoChooser.addOption("Path Planner Test Auto", () -> PathPlannerAutos.pathplannerAuto("TestPath", swerveDrive));
    autoChooser.addOption("Path Planner Charge Around LEFT", () -> PathPlannerAutos.pathplannerAuto("ChargeAroundLEFT", swerveDrive));
    autoChooser.addOption("Path Planner TaxiRIGHT", () -> PathPlannerAutos.pathplannerAuto("TaxiRIGHT", swerveDrive));
    autoChooser.addOption("Path Planner TaxiLEFT", () -> PathPlannerAutos.pathplannerAuto("TaxiLEFT", swerveDrive));

    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");

    autosTab.add("Selected Auto", autoChooser);
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
    Command currentAuto = autoChooser.getSelected().get();

    swerveDrive.setDriveMode(DRIVE_MODE.AUTONOMOUS);
    return currentAuto;
  }
}
