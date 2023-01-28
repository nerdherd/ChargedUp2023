// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;

public class Arm extends SubsystemBase {
    private DoubleSolenoid arm;
    private TalonSRX rotatingArm;
    private boolean armExtended = false;
    private int targetTicks;

    public Arm() {
        arm = new DoubleSolenoid(ClawConstants.kPCMPort, PneumaticsModuleType.CTREPCM, ArmConstants.kPistonForwardID, ArmConstants.kPistonReverseID);
        rotatingArm = new TalonSRX(ArmConstants.kRotatingArmID);

        CommandScheduler.getInstance().registerSubsystem(this);
        initShuffleboard();
        rotatingArm = new TalonSRX(ArmConstants.kRotatingArmID);

        rotatingArm.setInverted(false);

        rotatingArm.config_kP(0, ArmConstants.kArmP);
        rotatingArm.config_kI(0, ArmConstants.kArmI);
        rotatingArm.config_kD(0, ArmConstants.kArmD);

        rotatingArm.configMotionCruiseVelocity(ArmConstants.kArmCruiseVelocity);
        rotatingArm.configMotionAcceleration(ArmConstants.kArmMotionAcceleration);
    }

    public void moveArmJoystick(double currentJoystickOutput) {
        double armTicks = rotatingArm.getSelectedSensorPosition();

        if (currentJoystickOutput >= 0.5 ) {
            currentJoystickOutput = 0.5;
        }

        if (currentJoystickOutput > ArmConstants.kArmDeadband) {
            rotatingArm.set(ControlMode.PercentOutput, 
                ((currentJoystickOutput * ArmConstants.kJoystickMultiplier)));
        } else if (currentJoystickOutput < -ArmConstants.kArmDeadband) {
            rotatingArm.set(ControlMode.PercentOutput, 
                ((currentJoystickOutput * ArmConstants.kJoystickMultiplier)));
        } else {
            rotatingArm.set(ControlMode.PercentOutput, 0);
        }

    }

    public void moveArmMotionMagic(int position) {
        // config tuning params in slot 0
        rotatingArm.set(ControlMode.MotionMagic, position);
        targetTicks = position;

    }

    public void setPowerZero() {
        rotatingArm.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void periodic() {}

    public CommandBase moveArmScore() {
        return runOnce(
            () -> {
                moveArmMotionMagic(ArmConstants.kArmScore);
            }
        );

    }

    public CommandBase moveArmStow() {
        return runOnce(
            () -> {
                moveArmMotionMagic(ArmConstants.kArmStow);;
            }
        );

    }

    public CommandBase armStow() {
        return runOnce(
            () -> {
                arm.set(Value.kReverse);
                armExtended = false;
            }
        );
    }

    public CommandBase armExtend() {
        return runOnce(
            () -> {
                arm.set(Value.kForward);
                armExtended = true;
            }
        );
    }

    private void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Arm");
        
        tab.addBoolean("Arm Extended", () -> armExtended);
        tab.addNumber("Current Arm Ticks", () -> rotatingArm.getSelectedSensorPosition());
        tab.addNumber("Target Arm Ticks", () -> targetTicks);
    }
}

  

