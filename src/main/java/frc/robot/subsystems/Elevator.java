// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.NerdyMath;

public class Elevator extends SubsystemBase implements Reportable{

  private TalonFX elevator;
  private int targetTicks;
  public BooleanSupplier atTargetPosition;
  public DoubleSupplier percentExtended;
  // private DigitalInput limitSwitch;

  /** Creates a new Elevator. */
  public Elevator() {
    elevator = new TalonFX(ElevatorConstants.kElevatorID);
    elevator.setInverted(false);
    atTargetPosition = () -> (NerdyMath.inRange(elevator.getSelectedSensorPosition(), targetTicks - 1500, targetTicks + 1500));
    // limitSwitch = new DigitalInput(ElevatorConstants.kLimitSwitchID);

    percentExtended = () -> (elevator.getSelectedSensorPosition() / (ElevatorConstants.kElevatorScoreHigh + 1500));
    SmartDashboard.putNumber("Elevator kP", ElevatorConstants.kElevatorP);
    SmartDashboard.putNumber("Elevator kI", ElevatorConstants.kElevatorI);
    SmartDashboard.putNumber("Elevator kD", ElevatorConstants.kElevatorD);
    SmartDashboard.putNumber("Elevator kF", ElevatorConstants.kElevatorF);
    SmartDashboard.putNumber("Elevator Accel", ElevatorConstants.kElevatorMotionAcceleration);
    SmartDashboard.putNumber("Elevator Cruise Vel", ElevatorConstants.kElevatorCruiseVelocity);
  }


  public void moveElevatorJoystick(double currentJoystickOutput, double angle) {
    setBrakeMode();
        if (currentJoystickOutput > ElevatorConstants.kElevatorDeadband) {
          if (percentExtended.getAsDouble() >= 100) {
            elevator.set(ControlMode.PercentOutput, -ElevatorConstants.kArbitraryFF * Math.sin(angle));
          } else {
            elevator.set(ControlMode.PercentOutput, 0.40);
            // elevator.setNeutralMode(NeutralMode.Coast);
          }//((currentJoystickOutput * ArmConstants.kJoystickMultiplier)));
        } else if (currentJoystickOutput < -ElevatorConstants.kElevatorDeadband) {
          if (percentExtended.getAsDouble() <= 0) {
            elevator.set(ControlMode.PercentOutput, 0);
          } else {
            elevator.set(ControlMode.PercentOutput, -0.40);
            // elevator.setNeutralMode(NeutralMode.Coast);
          }
                //((currentJoystickOutput * ArmConstants.kJoystickMultiplier)));
        } else {
          // if (limitSwitch.get()) {
          if (percentExtended.getAsDouble() <= 0){
            elevator.set(ControlMode.PercentOutput, 0);
          } else {
            elevator.set(ControlMode.PercentOutput, -ElevatorConstants.kArbitraryFF * Math.sin(angle));

          }
          
        }

    }

    
  public void moveMotionMagic(double angle) {
    elevator.config_kP(0, SmartDashboard.getNumber("Elevator kP", ElevatorConstants.kElevatorP));
    elevator.config_kI(0, SmartDashboard.getNumber("Elevator kI", ElevatorConstants.kElevatorI));
    elevator.config_kD(0, SmartDashboard.getNumber("Elevator kD", ElevatorConstants.kElevatorD));
    elevator.config_kF(0, SmartDashboard.getNumber("Elevator kF", ElevatorConstants.kElevatorF));
    elevator.configMotionAcceleration(SmartDashboard.getNumber("Elevator Accel", ElevatorConstants.kElevatorMotionAcceleration));
    elevator.configMotionCruiseVelocity(SmartDashboard.getNumber("Elevator Cruise Vel", ElevatorConstants.kElevatorCruiseVelocity));
    double ff = ElevatorConstants.kArbitraryFF * Math.sin(angle);
    elevator.set(ControlMode.MotionMagic, targetTicks, DemandType.ArbitraryFeedForward, ff);
  }
  

  public void moveMotionMagic(int targetTicks, double angle) {
    this.targetTicks = targetTicks;

    elevator.config_kP(0, SmartDashboard.getNumber("Elevator kP", ElevatorConstants.kElevatorP));
    elevator.config_kI(0, SmartDashboard.getNumber("Elevator kI", ElevatorConstants.kElevatorI));
    elevator.config_kD(0, SmartDashboard.getNumber("Elevator kD", ElevatorConstants.kElevatorD));
    elevator.config_kF(0, SmartDashboard.getNumber("Elevator kF", ElevatorConstants.kElevatorF));
    elevator.configMotionAcceleration(SmartDashboard.getNumber("Elevator Accel", ElevatorConstants.kElevatorMotionAcceleration));
    elevator.configMotionCruiseVelocity(SmartDashboard.getNumber("Elevator Cruise Vel", ElevatorConstants.kElevatorCruiseVelocity));

    double ff = ElevatorConstants.kArbitraryFF * Math.sin(angle);
    elevator.set(ControlMode.MotionMagic, targetTicks, DemandType.ArbitraryFeedForward, ff);

    SmartDashboard.putNumber("FF", ff);
  }

  public CommandBase moveElevatorMid(double angle) {
    return Commands.run(
      () -> moveMotionMagic(ElevatorConstants.kElevatorScoreMid, angle)
    );
  }
  
  public CommandBase moveElevatorHigh(double angle) {
    return Commands.run(
      () -> moveMotionMagic(ElevatorConstants.kElevatorScoreHigh, angle)
    );
  }
  
  public CommandBase moveElevatorStow(double angle) {
    return Commands.run(
      () -> moveMotionMagic(ElevatorConstants.kElevatorStow, angle)
    );
  }

  public void setTargetTicks(int targetTicks) {
    this.targetTicks = targetTicks;
  }

  public void setPowerZero() {
    elevator.set(ControlMode.PercentOutput, 0);
  }

  public void setBrakeMode() {
    elevator.setNeutralMode(NeutralMode.Brake);
  }

  public void resetEncoder() {
    elevator.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void reportToSmartDashboard() {
    SmartDashboard.putNumber("Elevator Motor Output", elevator.getMotorOutputPercent());
    SmartDashboard.putNumber("Elevator Current", elevator.getStatorCurrent());
    SmartDashboard.putNumber("Elevator Current Ticks", elevator.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elevator Target Ticks", targetTicks);
    SmartDashboard.putNumber("Elevator Current Velocity", elevator.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Elevator Target Velocity", elevator.getActiveTrajectoryVelocity());
  }
}
