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

  private TalonSRX topMotor, bottomMotor;

  /** Creates a new MotorClaw. */
  public MotorClaw() {
    topMotor = new TalonSRX(ClawConstants.kTopMotorID);
    bottomMotor = new TalonSRX(ClawConstants.kBottomMotorID);

    topMotor.setInverted(true);
    bottomMotor.setInverted(true);

    bottomMotor.follow(topMotor);

    setNeutralMode(NeutralMode.Brake);
  }
  

  public CommandBase setPower(double power) {
    return runOnce(
      () -> {
        topMotor.set(ControlMode.PercentOutput, power);
        // bottomMotor.set(ControlMode.PercentOutput, power);
        setNeutralMode(NeutralMode.Brake);
      }
      
    );
  }

  public CommandBase setPower(double topPower, double bottomPower) {
    return runOnce(
      () -> {
        topMotor.set(ControlMode.PercentOutput, topPower);
        // bottomMotor.set(ControlMode.PercentOutput, bottomPower);
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
      waitSeconds(0.25),
      setPowerZero()
    );
  }

  public CommandBase intake() {
    return sequence(
      setPower(ClawConstants.kIntakePower),
      waitSeconds(0.25),
      setPower(ClawConstants.kIntakeNeutralPower)
    );
  }

  public void setNeutralMode(NeutralMode mode) {
    topMotor.setNeutralMode(mode);
    bottomMotor.setNeutralMode(mode);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void reportToSmartDashboard(LOG_LEVEL level) {
    switch (level) {
      case OFF:
          break;
      case ALL:
      case MEDIUM:
        SmartDashboard.putNumber("Motor Claw Velocity", topMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Motor Claw Current", topMotor.getStatorCurrent());
      case MINIMAL:
          break;
    }
  }

  public void initShuffleboard(LOG_LEVEL level) {
    ShuffleboardTab tab = Shuffleboard.getTab("Motor Claw");
    
    switch (level) {
      case OFF:
          break;
      case ALL:
      case MEDIUM:
        tab.addNumber("Velocity", topMotor::getSelectedSensorVelocity);
        tab.addNumber("Current", topMotor::getStatorCurrent);
      case MINIMAL:
          break;
    }
  }
}
