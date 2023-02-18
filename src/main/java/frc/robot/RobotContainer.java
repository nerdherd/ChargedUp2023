// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ApproachCombined;
import frc.robot.commands.DriveToTarget;
import frc.robot.subsystems.AirCompressor;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ConeRunner;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.TankDrivetrain;
import frc.robot.subsystems.Imu;
import frc.robot.subsystems.MotorClaw;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.commands.SwerveAutos;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.TankAutos;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.VisionCommands;
import frc.robot.commands.SwerveAutos.StartPosition;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.SwerveModuleType;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Limelight.LightMode;
import frc.robot.subsystems.vision.Vision.PipelineType;
import frc.robot.util.BadPS4;
import frc.robot.util.CommandBadPS4;
import pabeles.concurrency.ConcurrencyOps.NewInstance;

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
  public static Elevator elevator = new Elevator();
  public static Claw claw = new Claw();
  
  public static MotorClaw motorClaw = new MotorClaw();

  public static Imu imu = new Imu();
  public static Vision vision = new Vision();
  public static ConeRunner coneRunner = new ConeRunner();
  public static final boolean IsSwerveDrive = true;
  public static TankDrivetrain tankDrive;
  public static SwerveDrivetrain swerveDrive;
  public AirCompressor airCompressor = new AirCompressor();

  private PipelineType obj = PipelineType.ATAG;

  private final CommandBadPS4 driverController = new CommandBadPS4(
      ControllerConstants.kDriverControllerPort);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandBadPS4 operatorController = new CommandBadPS4(
      ControllerConstants.kOperatorControllerPort);
  private final BadPS4 badPS4 = operatorController.getHID();

  private final POVButton upButton = new POVButton(badPS4, 0);
  private final POVButton rightButton = new POVButton(badPS4, 90);
  private final POVButton downButton = new POVButton(badPS4, 180);
  private final POVButton leftButton = new POVButton(badPS4, 270);


  SendableChooser<CommandBase> autoChooser = new SendableChooser<CommandBase>();

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
      try {
        swerveDrive = new SwerveDrivetrain(imu, SwerveModuleType.CANCODER);
      } catch (IllegalArgumentException e) {
        DriverStation.reportError("Illegal Swerve Drive Module Type", e.getStackTrace());
      }

      swerveCommand = new RepeatCommand(
          new SequentialCommandGroup(
              new WaitCommand(5),
              new InstantCommand(swerveDrive::resetEncoders)));

      autoChooser.setDefaultOption("Pickup Cone Auto", SwerveAutos.twoPieceChargeAuto(swerveDrive, arm, elevator, claw, StartPosition.Right));
      autoChooser.addOption("Hard Carry", SwerveAutos.hardCarryAuto(swerveDrive));
      autoChooser.addOption("Vending Machine", SwerveAutos.vendingMachine(swerveDrive));
      autoChooser.addOption("Test auto", SwerveAutos.twoPieceChargeAuto(swerveDrive, arm, elevator, claw, StartPosition.Right));
      SmartDashboard.putData(autoChooser);
      SmartDashboard.putData("Encoder reset", Commands.runOnce(swerveDrive::resetEncoders, swerveDrive));

    } else {
      tankDrive = new TankDrivetrain();

      // visionRunCommand = new RunCommand(
      //     () -> tankDrive.arcadeDrive(tankDrive.getApriltagLinear(), tankDrive.getApriltagRotation()), tankDrive);
    }


    // Configure the trigger bindings
    configureBindings();
  }

  public void initDefaultCommands() {
    arm.armExtend();
    arm.setDefaultCommand(
      new RunCommand(
        () -> {
          arm.moveArmJoystick(operatorController.getLeftY(), elevator.percentExtended.getAsDouble());
          SmartDashboard.putNumber("Arm input", operatorController.getLeftY());
        }, 
        arm
      ));
    
    arm.resetEncoder();

    elevator.setDefaultCommand(
      new RunCommand(
        () -> {
          elevator.moveElevatorJoystick(operatorController.getRightY(), arm.armAngle.getAsDouble());
          SmartDashboard.putNumber("Elevator input", operatorController.getRightY());
        }, 
        elevator
      ));

    elevator.resetEncoder();

    coneRunner.setDefaultCommand(
      Commands.run(() -> {
        coneRunner.joystickSpeedControl(0.5*operatorController.getLeftY());
        coneRunner.joystickAngleControl(0.5*operatorController.getRightY());
      }, coneRunner)
    );

    coneRunner.resetEncoders();
    // arm.setDefaultCommand(arm.moveArmJoystickCommand(operatorController::getLeftY));

    if (IsSwerveDrive) {
      swerveDrive.setDefaultCommand(
        new SwerveJoystickCommand(
          swerveDrive,
          () -> -driverController.getLeftY(),
          driverController::getLeftX,
          // () -> 0.0,
          driverController::getRightX,
          // () -> true,
          badPS4::getSquareButton,
          badPS4::getL3Button,
          // driverControllerButtons::getTriangleButton,
          badPS4::getR3Button
        ));
    } else {
      tankDrive.setDefaultCommand(
        new RunCommand(
          () -> tankDrive.drive(
            -driverController.getLeftY(), 
            -driverController.getRightY()
          ), tankDrive));
    }


  }

  private void configureBindings() {
    // Note: whileTrue() does not restart the command if it ends while the button is
    // still being held
    // These button bindings are chosen for testing, and may be changed based on
    // driver preference
    if (!IsSwerveDrive) {
      driverController.L1().whileTrue(tankDrive.shiftHigh()); // TODO: use it for swerve too? inch-drive
      driverController.R1().whileTrue(tankDrive.shiftLow());
    }

    
    upButton.whileTrue(arm.moveArmStow()) 
      .onFalse(Commands.runOnce(arm::setPowerZero));
    leftButton.whileTrue(arm.moveArmScore()) 
      .onFalse(Commands.runOnce(arm::setPowerZero));
    downButton.whileTrue(arm.moveArmGround()) 
      .onFalse(Commands.runOnce(arm::setPowerZero));
    
    operatorController.triangle().whileTrue(elevator.moveElevatorHigh(arm.armAngle.getAsDouble()))
      .onFalse(Commands.runOnce(elevator::setPowerZero));
    operatorController.square().whileTrue(elevator.moveElevatorMid(arm.armAngle.getAsDouble()))
      .onFalse(Commands.runOnce(elevator::setPowerZero));
    operatorController.cross().whileTrue(elevator.moveElevatorStow(arm.armAngle.getAsDouble()))
      .onFalse(Commands.runOnce(elevator::setPowerZero));
  
    operatorController.share().onTrue(Commands.runOnce(arm::resetEncoder));
    operatorController.options().onTrue(Commands.runOnce(elevator::resetEncoder));
    
    
    // operatorController.triangle().whileTrue(arm.armExtend());
    // operatorController.square().whileTrue(arm.armStow());
    // operatorController.L1().whileTrue(motorClaw.setPower(0.4))
    //     .onFalse(motorClaw.setPowerZero());
    // operatorController.R1().whileTrue(motorClaw.setPower(-0.4))
    //     .onFalse(motorClaw.setPowerZero());
    // operatorController.circle().onTrue(claw.clawOpen());
    // operatorController.cross().onTrue(claw.clawClose());

    // operatorController.R1().whileTrue(claw.clawOpen()).onFalse(claw.clawClose());
    // operatorController.L1().whileTrue(arm.armExtend()).onFalse(arm.armStow());

    if (IsSwerveDrive) {
      // Driver Bindings
      driverController.share().onTrue(new InstantCommand(imu::zeroHeading));
      driverController.options().onTrue(new InstantCommand(swerveDrive::resetEncoders));

      driverController.R1().whileTrue(new TurnToAngle(180, swerveDrive));
      driverController.L1().whileTrue(new TurnToAngle(0, swerveDrive));

      driverController.triangle().whileTrue(VisionCommands.penPineappleApplePen(swerveDrive, vision))
                      .onFalse(new InstantCommand(() -> swerveDrive.stopModules()));

      // Operator Bindings
      // operatorController.R1().onTrue(vision.SwitchHigh());
      // operatorController.L1().onTrue(vision.SwitchLow());



      // driverController.triangle().whileTrue(new DriveToTarget(swerveDrive, objDetectCamera, 4, obj))
      //                  .onFalse(Commands.runOnce(swerveDrive::stopModules, swerveDrive));

      //driverController.triangle().whileTrue(new ApproachCombined(swerveDrive, objDetectCamera, 4, obj))
      //.onFalse(Commands.runOnce(swerveDrive::stopModules, swerveDrive));
    }
  }
  
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

  public void reportAllToSmartDashboard() {
    // SmartDashboard.putNumber("Timestamp", WPIUtilJNI.now());
    imu.reportToSmartDashboard();
    claw.reportToSmartDashboard();
    arm.reportToSmartDashboard();
    coneRunner.reportToSmartDashboard();
    if (IsSwerveDrive) {
      swerveDrive.reportToSmartDashboard();
      swerveDrive.reportModulesToSmartDashboard();
    } else {
      tankDrive.reportToSmartDashboard();
    }
    airCompressor.reportToSmartDashboard();
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // if (IsSwerveDrive)
    //   return SwerveAutos.twoPieceChargeAuto(swerveDrive, arm, claw, StartPosition.Right);
    // else
    //   return TankAutos.HardCarryAuto(tankDrive, claw, arm);
  }

  public void autonomousInit() {
    if (!IsSwerveDrive) { // TODO: Move resets to robot init? 
      tankDrive.resetEncoders();
      // drive.setEncoder(drive.meterToTicks(0.381));
      imu.zeroHeading();

    }
    
    arm.resetEncoder();
    arm.setTargetTicks(ArmConstants.kArmStow);
    elevator.resetEncoder();
    elevator.setTargetTicks(ElevatorConstants.kElevatorStow);

    // if (IsSwerveDrive) {
    //   swerveDrive.resetEncoders();
    // }
  }
}
