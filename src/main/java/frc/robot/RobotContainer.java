// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.AirCompressor;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ConeRunner;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Imu;
import frc.robot.subsystems.MotorClaw;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
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
import frc.robot.subsystems.vision.VROOOOM;
import frc.robot.subsystems.vision.VROOOOM.OBJECT_TYPE;
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

  public static Arm arm = new Arm();
  public static Elevator elevator = new Elevator();
  // public static Claw claw = new Claw();
  
  public static MotorClaw motorClaw = new MotorClaw();

  public static Imu imu = new Imu();
  // public static ConeRunner coneRunner = new ConeRunner();
  //public static final boolean IsSwerveDrive = true;
  //public static TankDrivetrain tankDrive;
  public static SwerveDrivetrain swerveDrive;
  // public AirCompressor airCompressor = new AirCompressor();
  public VROOOOM vision;

  private final CommandBadPS4 driverController = new CommandBadPS4(
      ControllerConstants.kDriverControllerPort);
  private final BadPS4 badPS5 = driverController.getHID();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandBadPS4 operatorController = new CommandBadPS4(
      ControllerConstants.kOperatorControllerPort);
  private final BadPS4 badPS4 = operatorController.getHID();
  // private final Joystick joystick = new Joystick(2);

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
        swerveDrive = new SwerveDrivetrain(imu, SwerveModuleType.CANCODER);
        vision = new VROOOOM(arm, elevator, motorClaw, swerveDrive);
      } catch (IllegalArgumentException e) {
        DriverStation.reportError("Illegal Swerve Drive Module Type", e.getStackTrace());
      }

      // Initialize vision after swerve has been initialized
      vision = new VROOOOM(arm, elevator, motorClaw, swerveDrive);

      // this.alliance = DriverStation.getAlliance();
      initAutoChoosers();
      
      SmartDashboard.putData("Encoder reset", Commands.runOnce(swerveDrive::resetEncoders, swerveDrive));

    // } else {
    //   tankDrive = new TankDrivetrain();

    // }

    // elevator.resetEncoderStow();
    // Configure the trigger bindings
    configureBindings();
    
  }

  public void initDefaultCommands() {
    arm.resetEncoderStow();
    // elevator.resetEncoder();

    arm.setDefaultCommand(
      new RunCommand(
        () -> {
          arm.moveArmJoystick(operatorController.getLeftY(), elevator.percentExtended());
          SmartDashboard.putNumber("Arm input", operatorController.getLeftY());
        }, 
        arm
      ));
    
    // arm.resetEncoder();

    elevator.setDefaultCommand(
      new RunCommand(
        () -> {
          elevator.moveElevatorJoystick(operatorController.getRightY() * -0.125, arm.getArmAngle());
          SmartDashboard.putNumber("Elevator input", operatorController.getRightY());
        }, 
        elevator
      ));

    

    // coneRunner.setDefaultCommand(
    //   Commands.run(() -> {
    //     coneRunner.joystickAngleControl((operatorController.getR2Axis()+operatorController.getL2Axis())*0.2 / 2);
    //   }, coneRunner)
    // );


    // coneRunner.resetEncoders();
    // arm.setDefaultCommand(arm.moveArmJoystickCommand(operatorController::getLeftY));

    //if (IsSwerveDrive) {
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
    // } else {
    //   tankDrive.setDefaultCommand(
    //     new RunCommand(
    //       () -> tankDrive.drive(
    //         -driverController.getLeftY(), 
    //         -driverController.getRightY()
    //       ), tankDrive));
    // }


  }

  private void configureBindings() {
    // Note: whileTrue() does not restart the command if it ends while the button is
    // still being held
    // These button bindings are chosen for testing, and may be changed based on
    // driver preference
    // if (!IsSwerveDrive) {
    //   driverController.L1().whileTrue(tankDrive.shiftHigh()); // TODO: use it for swerve too? inch-drive
    //   driverController.R1().whileTrue(tankDrive.shiftLow());
    // }

    
    upButton.whileTrue(arm.moveArmStow(elevator::percentExtended)) 
      .onFalse(Commands.runOnce(arm::setArmPowerZero));
    leftButton.whileTrue(arm.moveArmScore(elevator::percentExtended)) 
      .onFalse(Commands.runOnce(arm::setArmPowerZero));
    rightButton.whileTrue(arm.moveArm(ArmConstants.kArmSubstation, elevator::percentExtended))
      .onFalse(Commands.runOnce(arm::setArmPowerZero));
    downButton.whileTrue(arm.moveArmGround(elevator::percentExtended)) 
      .onFalse(Commands.runOnce(arm::setArmPowerZero));
    
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
    
    // operatorController.triangle()
    //   .whileTrue(Commands.runOnce(() -> coneRunner.joystickSpeedControl(0.3)))
    //   .onFalse(Commands.runOnce(() -> coneRunner.joystickSpeedControl(0)));
    
    // operatorController.square()
    //   .whileTrue(Commands.runOnce(() -> coneRunner.joystickSpeedControl(-0.3)))
    //   .onFalse(Commands.runOnce(() -> coneRunner.joystickSpeedControl(0)));
    
    // operatorController.triangle().whileTrue(arm.armExtend());
    // operatorController.square().whileTrue(arm.armStow());
    operatorController.L1().whileTrue(motorClaw.setPower(1, 1))
        .onFalse(motorClaw.setPowerZero());
    operatorController.R1().whileTrue(motorClaw.setPower(-0.30))
        .onFalse(motorClaw.setPower(-0.2));
    // operatorController.circle().onTrue(claw.clawOpen());
    // operatorController.cross().onTrue(claw.clawClose());

    // operatorController.R1().whileTrue(claw.clawOpen()).onFalse(claw.clawClose());
    // operatorController.L1().whileTrue(arm.armExtend()).onFalse(arm.armStow());

    //if (IsSwerveDrive) {
      // Driver Bindings
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


      //operatorController.L2().onTrue(vision.updateCurrentGameObject(OBJECT_TYPE.CONE));
      //operatorController.R2().onTrue(vision.updateCurrentGameObject(OBJECT_TYPE.CUBE));

      //upButtonDriver.onTrue(vision.updateCurrentHeight(SCORE_POS.HIGH));
      //rightButtonDriver.onTrue(vision.updateCurrentHeight(SCORE_POS.MID));
      //downButtonDriver.onTrue(vision.updateCurrentHeight(SCORE_POS.LOW));

      upButtonDriver.whileTrue(vision.VisionScore(OBJECT_TYPE.CONE, SCORE_POS.HIGH))
      .onFalse(Commands.runOnce(swerveDrive::stopModules, swerveDrive));
      leftButtonDriver.whileTrue(vision.VisionScore(OBJECT_TYPE.CONE, SCORE_POS.MID))
        .onFalse(Commands.runOnce(swerveDrive::stopModules, swerveDrive));

      
      rightButtonDriver.whileTrue(vision.VisionScore(OBJECT_TYPE.CUBE, SCORE_POS.HIGH))
        .onFalse(Commands.runOnce(swerveDrive::stopModules, swerveDrive));
      downButtonDriver.whileTrue(vision.VisionScore(OBJECT_TYPE.CUBE, SCORE_POS.MID))
      .onFalse(Commands.runOnce(swerveDrive::stopModules, swerveDrive));

    //}
  }

  private void initAutoChoosers() {
    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");
    SmartDashboard.putBoolean("Dummy Auto", false);

    autoChooser.addOption("Charge only", () -> SwerveAutos.chargeAuto(swerveDrive, startPos, alliance, 0, false));
    autoChooser.addOption("Backward Auto", () -> SwerveAutos.driveBackwardAuto(swerveDrive));
    autoChooser.addOption("Preload Auto", () -> SwerveAutos.preloadAuto(arm, elevator, motorClaw, scorePos));
    autoChooser.addOption("Preload Charge Auto", () -> SwerveAutos.preloadChargeAuto(swerveDrive, arm, elevator, motorClaw, startPos, scorePos, 0, false, alliance));
    autoChooser.addOption("Preload Charge Go Around Auto", () -> SwerveAutos.preloadChargeAuto(swerveDrive, arm, elevator, motorClaw, startPos, scorePos, 0, true, alliance));
    autoChooser.addOption("Preload Backward Auto", () -> SwerveAutos.preloadBackwardAuto(swerveDrive, arm, elevator, motorClaw, startPos, scorePos, alliance));
    autoChooser.addOption("Preload Pickup Auto", () -> SwerveAutos.twoPieceAuto(swerveDrive, arm, elevator, motorClaw, startPos, scorePos, alliance));
    autoChooser.addOption("Preload Pickup Charge Auto", () -> SwerveAutos.twoPieceChargeAuto(swerveDrive, arm, elevator, motorClaw, startPos, scorePos, 0, false, alliance));
    autoChooser.addOption("Preload Pickup Charge Go Around Auto", () -> SwerveAutos.twoPieceChargeAuto(swerveDrive, arm, elevator, motorClaw, startPos, scorePos, 0, true, alliance));
    autoChooser.addOption("Preload Pickup Backward Auto", () -> SwerveAutos.twoPieceBackwardAuto(swerveDrive, arm, elevator, motorClaw, startPos, scorePos, alliance));
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
    imu.initShuffleboard();
    // claw.initShuffleboard();
    arm.initShuffleboard();
    elevator.initShuffleboard();
    // coneRunner.initShuffleboard();
    //if (IsSwerveDrive) {
      // swerveDrive.initShuffleboard();
      swerveDrive.initModuleShuffleboard();
    // } else {
    //   tankDrive.initShuffleboard();
    // }
    // airCompressor.initShuffleboard();

    vision.initShuffleboard();
  }

  public void reportAllToSmartDashboard() {
    SmartDashboard.putNumber("Elevator FF", Math.sin(arm.getArmAngle()) * ElevatorConstants.kArbitraryFF);
    SmartDashboard.putNumber("Arm FF", -(ArmConstants.kStowedFF + ArmConstants.kDiffFF * elevator.percentExtended()) * Math.cos(arm.getArmAngle()));
    // SmartDashboard.putNumber("Timestamp", WPIUtilJNI.now());
    imu.reportToSmartDashboard();
    // claw.reportToSmartDashboard();
    arm.reportToSmartDashboard();
    elevator.reportToSmartDashboard();
    vision.reportToSmartDashboard();
    // coneRunner.reportToSmartDashboard();
    // if (IsSwerveDrive) {
      // swerveDrive.reportToSmartDashboard();
      // swerveDrive.reportModulesToSmartDashboard();
    // } else {
    //   tankDrive.reportToSmartDashboard();
    // }
    // airCompressor.reportToSmartDashboard();
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
    // Command currentAuto = SwerveAutos.backupChargeAuto(swerveDrive);
    // String autoName = currentAuto.getName();
    if (currentAuto != null) {
      // Shuffleboard.getTab("Autos").addString("Current Auto", () -> autoName);
    }
    return currentAuto;
    // if (IsSwerveDrive)
    //   return SwerveAutos.twoPieceChargeAuto(swerveDrive, arm, claw, StartPosition.Right);
    // else
    //   return TankAutos.HardCarryAuto(tankDrive, claw, arm);
  }

  public void autonomousInit() {
    // if (!IsSwerveDrive) { // TODO: Move resets to robot init? 
    //   tankDrive.resetEncoders();
    //   // drive.setEncoder(drive.meterToTicks(0.381));
    imu.zeroHeading();
    

    // }
    
    arm.resetEncoderStow();
    elevator.resetEncoder();

    // if (IsSwerveDrive) {
    //   swerveDrive.resetEncoders();
    // }
  }
}
