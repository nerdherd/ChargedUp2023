// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class MotorClaw extends SubsystemBase implements Reportable {

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

  public CommandBase outtake() {
    return sequence(
      setPower(ClawConstants.kOuttakePower),
      waitSeconds(1),
      setPower(ClawConstants.kIntakeNeutralPower)
    );
  }

  public CommandBase intake() {
    return sequence(
      setPower(ClawConstants.kIntakePower),
      waitSeconds(1),
      setPowerZero()
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
