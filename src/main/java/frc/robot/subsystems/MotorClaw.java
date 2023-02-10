// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class MotorClaw extends SubsystemBase {

  private TalonSRX leftMotor, rightMotor;

  /** Creates a new MotorClaw. */
  public MotorClaw() {
    leftMotor = new TalonSRX(ClawConstants.kLeftMotorID);
    rightMotor = new TalonSRX(ClawConstants.kRightMotorID);

    leftMotor.setInverted(false);
    rightMotor.setInverted(true);
  }

  public CommandBase setPower(double power) {
    return runOnce(
      () -> {
        leftMotor.set(ControlMode.PercentOutput, power);
        rightMotor.set(ControlMode.PercentOutput, power);
      }
      
    );
  }

  public CommandBase setPowerZero() {
    return setPower(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
