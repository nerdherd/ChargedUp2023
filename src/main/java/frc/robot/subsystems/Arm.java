// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ControllerConstants;

public class Arm extends SubsystemBase {
    private DoubleSolenoid arm;

    public Arm() {
        arm = new DoubleSolenoid(ClawConstants.kPCMPort, PneumaticsModuleType.CTREPCM, ArmConstants.kPistonForwardID, ArmConstants.kPistonReverseID);

    }

    public CommandBase armStow() {
        return runOnce(
            () -> {
                arm.set(Value.kReverse);
            }
        );
    }

    public CommandBase armExtend() {
        return runOnce(
            () -> {
                arm.set(Value.kForward);
            }
        );
    }
}
