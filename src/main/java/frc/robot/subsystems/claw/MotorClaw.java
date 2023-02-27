// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Reportable;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class MotorClaw extends SubsystemBase implements Reportable, Claw {

  private TalonSRX leftMotor, rightMotor;

  /** Creates a new MotorClaw. */
  public MotorClaw() {
    leftMotor = new TalonSRX(ClawConstants.kLeftMotorID);
    rightMotor = new TalonSRX(ClawConstants.kRightMotorID);

    leftMotor.setInverted(false);
    rightMotor.setInverted(true);

    setNeutralMode(NeutralMode.Brake);
  }
  

  public CommandBase setPower(double power) {
    return runOnce(
      () -> {
        leftMotor.set(ControlMode.PercentOutput, power);
        rightMotor.set(ControlMode.PercentOutput, power);
        setNeutralMode(NeutralMode.Brake);
      }
      
    );
  }

  public CommandBase setPowerZero() {
    return setPower(0);
  }

  public CommandBase outtakeCube() {
    return sequence(
      setPower(ClawConstants.kCubeOuttakePower),
      waitSeconds(1),
      setPowerZero()
    );
  }

  public CommandBase outtakeCone() {
    return sequence(
      setPower(ClawConstants.kConeOuttakePower),
      waitSeconds(1),
      setPowerZero()
    );
  }

  public CommandBase intakeCube() {
    return sequence(
      setPower(ClawConstants.kCubeIntakePower),
      waitSeconds(1),
      setPower(ClawConstants.kCubeHoldPower)
    );
  }

  public CommandBase intakeCone() {
    return sequence(
      setPower(ClawConstants.kConeIntakePower),
      waitSeconds(1),
      setPower(ClawConstants.kConeHoldPower)
    );
  }

  public void setNeutralMode(NeutralMode mode) {
    leftMotor.setNeutralMode(mode);
    rightMotor.setNeutralMode(mode);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void reportToSmartDashboard() {
    SmartDashboard.putNumber("Motor Claw Velocity", leftMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Motor Claw Current", leftMotor.getStatorCurrent());
  }

  public void initShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Motor Claw");
    tab.addNumber("Velocity", leftMotor::getSelectedSensorVelocity);
    tab.addNumber("Current", leftMotor::getStatorCurrent);
  }
}
