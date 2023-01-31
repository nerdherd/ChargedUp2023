// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriveToTarget;
import frc.robot.subsystems.AirCompressor;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ConeRunner;
import frc.robot.subsystems.TankDrivetrain;
import frc.robot.subsystems.Imu;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.commands.SwerveAutos;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.TankAutos;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.SwerveDrivetrain;

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

  public static Arm arm = new Arm();
  public static Claw claw = new Claw();
  public static Imu imu = new Imu();// AHRS(SPI.Port.kMXP);
  // public static Vision vision = new Vision();
  public static Limelight objDetectCamera = new Limelight();
  public static ConeRunner coneRunner = new ConeRunner();
  // public static Drivetrain drive;
  public static final boolean IsSwerveDrive = true;
  public static TankDrivetrain tankDrive;// = new TankDrivetrain(vision);
  public static SwerveDrivetrain swerveDrive;
  public AirCompressor airCompressor = new AirCompressor();

  private final CommandPS4Controller driverController = new CommandPS4Controller(
      ControllerConstants.kDriverControllerPort);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller operatorController = new CommandPS4Controller(
      ControllerConstants.kOperatorControllerPort);

  private final PS4Controller driverControllerButtons = new PS4Controller(ControllerConstants.kDriverControllerPort);

  SendableChooser<CommandBase> autoChooser = new SendableChooser<CommandBase>();

  public double swerveTargetAngle = 180;

  // Two different drivetrain modes
  private RunCommand arcadeRunCommand;
  private RunCommand visionRunCommand;
  
  // Two different drivetrain modes
  // private RunCommand arcadeRunCommand = new RunCommand(() -> drive.tankDrive(driverController.getLeftY(), driverController.getRightY()), drive);
  // private RunCommand visionRunCommand = new RunCommand(() -> drive.arcadeDrive(drive.getApriltagLinear(), drive.getApriltagRotation()), drive);
  // private RunCommand visionRunCommandArea = new RunCommand(() -> drive.arcadeDrive(drive.getAprilTagAreaLinear(), drive.getApriltagRotation()), drive);

  public Command swerveCommand;

  public SwerveJoystickCommand swerveJoystickCommand;
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    if (IsSwerveDrive) {
      swerveDrive = new SwerveDrivetrain(imu.ahrs);

      swerveCommand = new RepeatCommand(
          new SequentialCommandGroup(
              new WaitCommand(5),
              new InstantCommand(swerveDrive::resetEncoders)));

      swerveJoystickCommand = new SwerveJoystickCommand(swerveDrive,
          () -> -driverController.getLeftY(),
          driverController::getLeftX,
          // () -> 0.0,
          driverController::getRightY,
          driverControllerButtons::getSquareButton,
          () -> false,
          // driverControllerButtons::getTriangleButton,
          driverControllerButtons::getCrossButton);

      autoChooser.setDefaultOption("Hard Carry", SwerveAutos.hardCarryAuto(swerveDrive));
      autoChooser.addOption("Hard Carry", SwerveAutos.hardCarryAuto(swerveDrive));
      autoChooser.addOption("Vending Machine", SwerveAutos.vendingMachine(swerveDrive));
      autoChooser.addOption("Test auto", SwerveAutos.twoPieceChargeAuto(swerveDrive, arm, claw));
      SmartDashboard.putData(autoChooser);

      swerveDrive.setDefaultCommand(swerveJoystickCommand);
    } else {
      tankDrive = new TankDrivetrain();
      arcadeRunCommand = new RunCommand(
          () -> tankDrive.tankDrive(driverController.getLeftY(), driverController.getRightY()), tankDrive);

      // visionRunCommand = new RunCommand(
      //     () -> tankDrive.arcadeDrive(tankDrive.getApriltagLinear(), tankDrive.getApriltagRotation()), tankDrive);
    }
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // Note: whileTrue() does not restart the command if it ends while the button is
    // still being held
    // These button bindings are chosen for testing, and may be changed based on
    // driver preference
    // driverController.square().whileTrue(arm.armExtend());
    // driverController.triangle().whileTrue(arm.armStow());
    // driverController.circle().onTrue(claw.clawOpen());
    // driverController.cross().onTrue(claw.clawClose());
    if (!IsSwerveDrive) {
      driverController.L1().whileTrue(tankDrive.shiftHigh());
      driverController.R1().whileTrue(tankDrive.shiftLow());
    }
    operatorController.triangle().whileTrue(arm.armExtend());
    operatorController.square().whileTrue(arm.armStow());
    operatorController.circle().onTrue(claw.clawOpen());
    operatorController.cross().onTrue(claw.clawClose());

    // operatorController.L1().onTrue(arm.moveArmScore());
    // operatorController.R1().onTrue(arm.moveArmStow());

    // driverController.circle().onTrue(coneRunner.runConeRunner());
    // driverController.cross().onTrue(coneRunner.stopConeRunner());

    // driverController.circle().onFalse(arcadeRunCommand);
    // driverController.circle().whileTrue(visionRunCommand);
    // driverController.circle().onFalse(new InstantCommand(() ->
    // SmartDashboard.putBoolean("Vision Mode", false)));
    // driverController.circle().whileTrue(new InstantCommand(() ->
    // SmartDashboard.putBoolean("Vision Mode", true)));
    if (IsSwerveDrive) {
      driverController.circle().onTrue(new InstantCommand(swerveDrive::zeroHeading));
      driverController.square().onTrue(new InstantCommand(swerveDrive::resetEncoders));
      
      // driverController.cross().onTrue(new InstantCommand(() ->
      // swerveJoystickCommand.setTargetAngle((swerveJoystickCommand.getTargetAngle()
      // + 90) % 360)));
      // driverController.cross().onTrue(new InstantCommand(() ->
      // swerveDrive.resetOdometry(new Pose2d())));
      // SmartDashboard.p utData("Turn to 180 degrees", new TurnToAngle(180,
      // swerveDrive));
      // driverController.cross().whileTrue(new SequentialCommandGroup(
      // new TurnToAngle(swerveTargetAngle, swerveDrive),
      // new InstantCommand(() -> swerveTargetAngle = (swerveTargetAngle + 180) %
      // 360))
      // );
      driverController.R1().whileTrue(new TurnToAngle(180, swerveDrive));
      driverController.L1().whileTrue(new TurnToAngle(0, swerveDrive));
      driverController.triangle().whileTrue(new DriveToTarget(objDetectCamera, swerveDrive, 2));
    }

    // driverController.triangle().whileTrue(new TheGreatBalancingAct(swerveDrive));

    // driverController.R1().whileTrue(new DriveToTarget(objDetectCamera, swerveDrive, 5));
  }

  public void configurePeriodic() {
    arm.moveArmJoystick(operatorController.getLeftY());
    if (!IsSwerveDrive) {
      tankDrive.tankDrive(-driverController.getLeftY(), -driverController.getRightY());
    }
    claw.periodic();
    // arm.moveArmJoystick(operatorController.getLeftY());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public void initShuffleboard() {
    if (!IsSwerveDrive) {
      tankDrive.initShuffleboard();
    }
    // autoChooser = new SendableChooser<CommandBase>();
    // autoChooser.setDefaultOption("Hard Carry Auto",
    // TankAutos.HardCarryAuto(drive, claw, arm));

    // autoChooser.addOption("Diet Coke Auto",
    // TankAutos.DietCokeAuto(drive, claw, arm));
  }

  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    if (IsSwerveDrive)
      return SwerveAutos.twoPieceChargeAuto(swerveDrive, arm, claw);
    else
      return TankAutos.HardCarryAuto(tankDrive, claw, arm);
    // An example command will be run in autonomous
    // return SwerveAutos.testAuto(swerveDrive);
  }

  public void autonomousInit() {
    if (!IsSwerveDrive) {
      tankDrive.resetEncoder();
      // drive.setEncoder(drive.meterToTicks(0.381));
      tankDrive.zeroHeading();

    }
  }
}
