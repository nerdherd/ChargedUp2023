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
import frc.robot.subsystems.TankDrivetrain;
import frc.robot.subsystems.Imu;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.MotorClaw;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Limelight.LightMode;
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
import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.commands.SwerveAutos;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.TankAutos;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.VisionAutos;
import frc.robot.commands.SwerveAutos.StartPosition;
import frc.robot.subsystems.Vision.PipelineType;
import frc.robot.util.BadPS4;
import frc.robot.util.CommandBadPS4;
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
  
  public static MotorClaw motorClaw = new MotorClaw();

  public static Imu imu = new Imu();
  public static Vision vision = new Vision();
  public static ConeRunner coneRunner = new ConeRunner();
  public static final boolean IsSwerveDrive = true;
  public static TankDrivetrain tankDrive;
  public static SwerveDrivetrain swerveDrive;
  public AirCompressor airCompressor = new AirCompressor();

  private PipelineType obj = PipelineType.ATAG;

  // private final CommandPS4Controller driverController = new CommandPS4Controller(
  //     ControllerConstants.kDriverControllerPort);
  // // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandPS4Controller operatorController = new CommandPS4Controller(
  //     ControllerConstants.kOperatorControllerPort);

  private final CommandBadPS4 driverController = new CommandBadPS4(ControllerConstants.kDriverControllerPort);
  private final CommandBadPS4 operatorController = new CommandBadPS4(ControllerConstants.kOperatorControllerPort);

  // private final PS4Controller driverControllerButtons = new PS4Controller(ControllerConstants.kDriverControllerPort);

  private final BadPS4 driverControllerButtons = new BadPS4(ControllerConstants.kDriverControllerPort);

  private POVButton operatorPOVUp = new POVButton(driverControllerButtons, 0);
  private POVButton operatorPOVRight = new POVButton(driverControllerButtons, 90);
  private POVButton operatorPOVDown = new POVButton(driverControllerButtons, 180);
  private POVButton operatorPOVLeft = new POVButton(driverControllerButtons, 270);

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
      swerveDrive = new SwerveDrivetrain(imu);

      swerveCommand = new RepeatCommand(
          new SequentialCommandGroup(
              new WaitCommand(5),
              new InstantCommand(swerveDrive::resetEncoders)));

      autoChooser.setDefaultOption("Hard Carry", SwerveAutos.hardCarryAuto(swerveDrive));
      autoChooser.addOption("Hard Carry", SwerveAutos.hardCarryAuto(swerveDrive));
      autoChooser.addOption("Vending Machine", SwerveAutos.vendingMachine(swerveDrive));
      autoChooser.addOption("Test auto", SwerveAutos.twoPieceChargeAuto(swerveDrive, arm, claw, StartPosition.Right));

      // Add other Vision Autos later
      autoChooser.addOption("Pickup Cone Auto", VisionAutos.penPineappleApplePen(swerveDrive, vision.limelightLow));

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
          arm.moveArmJoystick(operatorController.getLeftY());
          SmartDashboard.putNumber("Arm input", operatorController.getLeftY());
        }, 
        arm
      ));
    
    arm.armResetEncoder();
    arm.elevatorResetEncoder();

    coneRunner.setDefaultCommand(
      Commands.run(() -> {
        coneRunner.joystickSpeedControl(0.5*operatorController.getLeftY());
        coneRunner.joystickAngleControl(0.15*operatorController.getRightY() + 0.15);
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
          driverControllerButtons::getSquareButton,
          driverControllerButtons::getL3Button,
          // driverControllerButtons::getTriangleButton,
          // driverControllerButtons::getCrossButton
          () -> false
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

    operatorPOVRight.whileTrue(arm.moveArmScore())
      .onFalse(Commands.runOnce(arm::setArmPowerZero));
    operatorPOVUp.whileTrue(arm.moveArmStow()) 
      .onFalse(Commands.runOnce(arm::setArmPowerZero));
    operatorPOVDown.whileTrue(arm.moveArmGround()) 
      .onFalse(Commands.runOnce(arm::setArmPowerZero));
    operatorPOVLeft.whileTrue(arm.moveArmPickUp())
      .onFalse(Commands.runOnce(arm::setArmPowerZero));

    operatorController.triangle().whileTrue(arm.moveElevatorScoreHigh())
      .onFalse(Commands.runOnce(arm::setElevatorPowerZero));
    operatorController.square().whileTrue(arm.moveElevatorScoreMid())
      .onFalse(Commands.runOnce(arm::setElevatorPowerZero));
    operatorController.cross().whileTrue(arm.moveElevatorStow())
      .onFalse(Commands.runOnce(arm::setElevatorPowerZero));
  
    // operatorController.circle().onTrue(arm.armExtend()); 
    // operatorController.cross().whileTrue(arm.armStow());
    operatorController.L2().whileTrue(motorClaw.setPower(0.4))
        .onFalse(motorClaw.setPowerZero());
    operatorController.R2().whileTrue(motorClaw.setPower(-0.4))
        .onFalse(motorClaw.setPowerZero());

    operatorController.L3().whileTrue(
      new RunCommand(
        () -> {
          arm.moveArmJoystick(operatorController.getLeftY());
          SmartDashboard.putNumber("Arm input", operatorController.getLeftY());
        }, 
        arm
      )
    );

    operatorController.R3().whileTrue(
      new RunCommand(
        () -> {
          arm.moveElevatorJoystick(operatorController.getRightY());
          SmartDashboard.putNumber("Elevator input", operatorController.getRightY());
        }, 
        arm
      )
    );
    // operatorController.square().onTrue(claw.clawOpen());
    // operatorController.triangle().onTrue(claw.clawClose());

    // operatorController.R1().whileTrue(claw.clawOpen()).onFalse(claw.clawClose());
    // operatorController.L1().whileTrue(arm.armExtend()).onFalse(arm.armStow());

    if (IsSwerveDrive) {
      // Driver Bindings
      driverController.share().onTrue(new InstantCommand(imu::zeroHeading));
      driverController.options().onTrue(new InstantCommand(swerveDrive::resetEncoders));

      driverController.R1().whileTrue(new TurnToAngle(180, swerveDrive));
      driverController.L1().whileTrue(new TurnToAngle(0, swerveDrive));

      // driverController.triangle().onTrue(new ApproachCombined(swerveDrive, 0, 2, PipelineType.CONE, vision.getLimelight()));  
      // driverController.square().onTrue(new ApproachCombined(swerveDrive, 0, 2, PipelineType.CUBE, vision.getLimelight()));      
      // driverController.circle().onTrue(new ApproachCombined(swerveDrive, 0, 2, PipelineType.TAPE, vision.getLimelight()));      
      // driverController.cross().onTrue(new ApproachCombined(swerveDrive, 0, 2, PipelineType.ATAG, vision.getLimelight())); 


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
    // return autoChooser.getSelected();
    if (IsSwerveDrive)
      return SwerveAutos.twoPieceChargeAuto(swerveDrive, arm, claw, StartPosition.Right);
    else
      return TankAutos.HardCarryAuto(tankDrive, claw, arm);
  }

  public void autonomousInit() {
    if (!IsSwerveDrive) { // TODO: Move resets to robot init? 
      tankDrive.resetEncoders();
      // drive.setEncoder(drive.meterToTicks(0.381));
      imu.zeroHeading();

    }
    
    arm.armResetEncoder();
    arm.setArmTargetTicks(ArmConstants.kArmStow);

    arm.elevatorResetEncoder();
    arm.setElevatorTargetTicks(ElevatorConstants.kElevatorStow);
    // if (IsSwerveDrive) {
    //   swerveDrive.resetEncoders();
    // }
  }
}
