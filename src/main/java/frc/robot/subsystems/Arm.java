// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;

public class Arm extends SubsystemBase {
    public TalonFX armMotor;

    public Arm() {
        armMotor = new TalonFX(ArmConstants.kArmID);
        armMotor.configMotionAcceleration(ArmConstants.kArmMotionAcceleration);
        armMotor.configMotionCruiseVelocity(ArmConstants.kArmCruiseVelocity);
        armMotor.configNeutralDeadband(ArmConstants.kArmDeadband);
        armMotor.config_kP(0, ArmConstants.kArmP);
        armMotor.config_kI(0, ArmConstants.kArmI);
        armMotor.config_kD(0, ArmConstants.kArmD);
    }

    public CommandBase armToMiddleNodePosition() {
        return runOnce(
            () -> {
                armMotor.set(ControlMode.MotionMagic, ArmConstants.kArmMiddleNode);
            });
      }

    public CommandBase armToTopNodePosition() {
        return runOnce(
            () -> {
                armMotor.set(ControlMode.MotionMagic, ArmConstants.kArmTopNode);
            });
    }

    public void movePercentOutput(double joystickOutput) {
        if (Math.abs(joystickOutput) > ControllerConstants.kOperatorJoystickDeadband) {
            armMotor.set(ControlMode.PercentOutput, joystickOutput);
        } else {
            armMotor.set(ControlMode.PercentOutput, 0);
        }
    }
  
}
