// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Reportable;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class MotorClaw extends SubsystemBase implements Reportable {

  // private TalonSRX topMotor, bottomMotor;
  private TalonFX mainMotor;

  /** Creates a new MotorClaw. */
  public MotorClaw() {
    // topMotor = new TalonSRX(ClawConstants.kTopMotorID);
    // bottomMotor = new TalonSRX(ClawConstants.kBottomMotorID);

    // topMotor.setInverted(false);
    // bottomMotor.setInverted(true);
    // bottomMotor.follow(topMotor);

    mainMotor = new TalonFX(ClawConstants.kMainMotorID);
    mainMotor.setInverted(false);

    setNeutralMode(NeutralMode.Brake);
    mainMotor.configVoltageCompSaturation(11);
    mainMotor.enableVoltageCompensation(true);
  }
  

  public CommandBase setPower(double power) {
    return runOnce(
      () -> {
        mainMotor.set(ControlMode.PercentOutput, power);
        // topMotor.set(ControlMode.PercentOutput, power);
        // bottomMotor.set(ControlMode.PercentOutput, power);
        setNeutralMode(NeutralMode.Brake);
      }
      
    );
  }

  // public CommandBase setPower(double topPower, double bottomPower) {
  //   return runOnce(
  //     () -> {
  //       topMotor.set(ControlMode.PercentOutput, topPower);
  //       // bottomMotor.set(ControlMode.PercentOutput, bottomPower);
  //       setNeutralMode(NeutralMode.Brake);
  //     }
      
  //   );
  // }

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
    // topMotor.setNeutralMode(mode);
    // bottomMotor.setNeutralMode(mode);
    mainMotor.setNeutralMode(mode);
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
        SmartDashboard.putNumber("Motor Claw Velocity", mainMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Motor Claw Current", mainMotor.getStatorCurrent());
      case MINIMAL:
          break;
    }
  }

  public void initShuffleboard(LOG_LEVEL level) {
    if (level == LOG_LEVEL.OFF || level == LOG_LEVEL.MINIMAL)  {
      return;
    }

    ShuffleboardTab tab = Shuffleboard.getTab("Motor Claw");
    
    switch (level) {
      case OFF:
          break;
      case ALL:
        // tab.addNumber("Top Motor Current", topMotor::getStatorCurrent);
        // tab.addNumber("Bottom Motor Current", bottomMotor::getStatorCurrent);
        tab.addNumber("Main Motor Current", mainMotor::getStatorCurrent);
      case MEDIUM:
        // tab.addNumber("Top Motor Voltage", topMotor::getMotorOutputVoltage);
        // tab.addNumber("Top Motor Percent Output", topMotor::getMotorOutputPercent);
        // tab.addNumber("Top Motor Velocity (ticks/100ms)", topMotor::getSelectedSensorVelocity);
        // tab.addNumber("Bottom Motor Voltage", bottomMotor::getMotorOutputVoltage);
        // tab.addNumber("Bottom Motor Percent Output", bottomMotor::getMotorOutputPercent);
        // tab.addNumber("Bottom Motor Velocity (ticks/100ms)", bottomMotor::getSelectedSensorVelocity);
        tab.addNumber("Main Motor Voltage", mainMotor::getMotorOutputVoltage);
        tab.addNumber("Main Motor Percent Output", mainMotor::getMotorOutputPercent);
        tab.addNumber("Main Motor Velocity (ticks/100ms)", mainMotor::getSelectedSensorVelocity);
      case MINIMAL:
          break;
    }
  }
}
