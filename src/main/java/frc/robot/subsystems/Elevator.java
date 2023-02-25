// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
  // private DigitalInput limitSwitch;

  /** Creates a new Elevator. */
  public Elevator() {
    elevator = new TalonFX(ElevatorConstants.kElevatorID);
    elevator.setNeutralMode(NeutralMode.Brake);
    elevator.setInverted(true);
    // limitSwitch = new DigitalInput(ElevatorConstants.kLimitSwitchID);

    atTargetPosition = () -> NerdyMath.inRange(elevator.getSelectedSensorPosition(), targetTicks - 1500, targetTicks + 1500);
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
            elevator.set(ControlMode.PercentOutput, 0.40);
            // elevator.setNeutralMode(NeutralMode.Coast);
          //((currentJoystickOutput * ArmConstants.kJoystickMultiplier)));
        } else if (currentJoystickOutput < -ElevatorConstants.kElevatorDeadband) {
          elevator.set(ControlMode.PercentOutput, -0.40);
            // elevator.setNeutralMode(NeutralMode.Coast);
      
                //((currentJoystickOutput * ArmConstants.kJoystickMultiplier)));
        } else {
          // if (limitSwitch.get()) {
          elevator.set(ControlMode.PercentOutput, 0);
          
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

    double ff = -ElevatorConstants.kArbitraryFF * Math.sin(angle);
    elevator.set(ControlMode.MotionMagic, targetTicks, DemandType.ArbitraryFeedForward, ff);

    SmartDashboard.putNumber("FF", ff);
  }

  public CommandBase moveElevator(int ticks, double angle) {
    return Commands.run(
      () -> moveMotionMagic(ticks, angle), this
      );
  }

  public CommandBase moveElevator(int ticks, Supplier<Double> angleSupplier) {
    return Commands.run(
      () -> moveMotionMagic(ticks, angleSupplier.get()), this
    );
  }

  public CommandBase moveElevatorMid(double angle) {
    return Commands.run(
      () -> moveMotionMagic(ElevatorConstants.kElevatorScoreMid, angle), this
    );
  }
  
  public CommandBase moveElevatorMid(Supplier<Double> angleSupplier) {
    return moveElevator(ElevatorConstants.kElevatorScoreMid, angleSupplier);
  }

  public CommandBase moveElevatorHigh(double angle) {
    return Commands.run(
      () -> moveMotionMagic(ElevatorConstants.kElevatorScoreHigh, angle), this
    );
  }

  public CommandBase moveElevatorHigh(Supplier<Double> angleSupplier) {
    return moveElevator(ElevatorConstants.kElevatorScoreHigh, angleSupplier);
  }
  
  public CommandBase moveElevatorStow(double angle) {
    return Commands.run(
      () -> moveMotionMagic(ElevatorConstants.kElevatorStow, angle), this
    );
  }

  public CommandBase moveElevatorStow(Supplier<Double> angleSupplier) {
    return moveElevator(ElevatorConstants.kElevatorStow, angleSupplier);
  }

  public double percentExtended() {
    return Math.abs(elevator.getSelectedSensorPosition() / (ElevatorConstants.kElevatorScoreHigh));
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
    SmartDashboard.putNumber("Elevator Percent Extended", percentExtended());
  }

  public void initShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Elevator");

    tab.addNumber("Motor Output", () -> elevator.getMotorOutputPercent());
    tab.addNumber("Current", () -> elevator.getStatorCurrent());
    tab.addNumber("Current Ticks", () -> elevator.getSelectedSensorPosition());
    tab.addNumber("Target Ticks", () -> targetTicks);
    tab.addNumber("Current Velocity", () -> elevator.getSelectedSensorVelocity());
    tab.addNumber("Target Velocity", () -> elevator.getActiveTrajectoryVelocity());
    tab.addNumber("Percent Extended", this::percentExtended);
  }
}
