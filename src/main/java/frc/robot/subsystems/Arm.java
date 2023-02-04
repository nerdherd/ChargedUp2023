// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PneumaticsConstants;

public class Arm extends SubsystemBase implements Reportable {
    private DoubleSolenoid arm;
    private TalonFX rotatingArm;
    private boolean armExtended = false;
    private int targetTicks;
    private PIDController armPID;

    public Arm() {
        arm = new DoubleSolenoid(PneumaticsConstants.kPCMPort, PneumaticsModuleType.CTREPCM, ArmConstants.kPistonForwardID, ArmConstants.kPistonReverseID);
        
        // gear ratio 27:1
        rotatingArm = new TalonFX(ArmConstants.kRotatingArmID);

        // CommandScheduler.getInstance().registerSubsystem(this);
        initShuffleboard();

        rotatingArm.setInverted(false);

        rotatingArm.config_kP(0, ArmConstants.kArmP);
        rotatingArm.config_kI(0, ArmConstants.kArmI);
        rotatingArm.config_kD(0, ArmConstants.kArmD);

        rotatingArm.configMotionCruiseVelocity(ArmConstants.kArmCruiseVelocity);
        rotatingArm.configMotionAcceleration(ArmConstants.kArmMotionAcceleration);
   
        armPID = new PIDController(
            SmartDashboard.getNumber("Arm kP", 0), 
            SmartDashboard.getNumber("Arm kI", 0), 
            SmartDashboard.getNumber("Arm kD", 0));
    }

    public void moveArmJoystick(double currentJoystickOutput) {
        // double armTicks = rotatingArm.getSelectedSensorPosition();


        if (currentJoystickOutput >= 0.75 ) {
            currentJoystickOutput = 0.75;
        }

        
        if (currentJoystickOutput > ArmConstants.kArmDeadband) {
            rotatingArm.set(ControlMode.PercentOutput, 0.75);
            rotatingArm.setNeutralMode(NeutralMode.Coast);
            //((currentJoystickOutput * ArmConstants.kJoystickMultiplier)));
        } else if (currentJoystickOutput < -ArmConstants.kArmDeadband) {
            rotatingArm.set(ControlMode.PercentOutput, -0.75);
            rotatingArm.setNeutralMode(NeutralMode.Coast);
                //((currentJoystickOutput * ArmConstants.kJoystickMultiplier)));
        } else {
            rotatingArm.set(ControlMode.PercentOutput, 0);
            rotatingArm.setNeutralMode(NeutralMode.Brake);
        }

    }

    public CommandBase moveArmJoystickCommand(Supplier<Double> joystickInput) {
        return Commands.run(
            () -> moveArmJoystickCommand(joystickInput), this);
    }

    public void moveArmMotionMagic(int position) {
        // config tuning params in slot 0
        rotatingArm.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, Math.cos(position)*ArmConstants.kArbitraryFF);
        targetTicks = position;

    }

    public void moveArm(double position) {
        armPID.setSetpoint(position);
        double speed = armPID.calculate(rotatingArm.getSelectedSensorPosition(), position);
        if (speed > 3000) {
            speed = 3000;
        } else if (speed < -3000) {
            speed = -3000;
        }
        rotatingArm.set(ControlMode.Velocity, speed);
        if (armPID.atSetpoint()) {
            rotatingArm.setNeutralMode(NeutralMode.Brake);
        } else {
            rotatingArm.setNeutralMode(NeutralMode.Coast);
        }
    }

    public void setPowerZero() {
        rotatingArm.set(ControlMode.PercentOutput, 0.0);
    }

    public void ticksToAngle() {
        
    }

    @Override
    public void periodic() {}

    public CommandBase moveArmScore() {
        return runOnce(
            () -> {
                moveArm(ArmConstants.kArmScore);
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

    public void reportToSmartDashboard() {
        SmartDashboard.putBoolean("Arm Extended", armExtended);
        SmartDashboard.putNumber("Current Arm Ticks", rotatingArm.getSelectedSensorPosition());
        SmartDashboard.putNumber("Target Arm Ticks", targetTicks);
    }
}

  

