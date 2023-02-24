// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.util.NerdyMath;

public class Arm extends SubsystemBase implements Reportable {
    private DoubleSolenoid arm;
    private TalonFX rotatingArm;
    private boolean armExtended = false;
    private int targetTicks = ArmConstants.kArmStow;
    private PIDController armPID;
    public BooleanSupplier atTargetPosition;
    private DigitalInput limitSwitch;

    public Arm() {
        arm = new DoubleSolenoid(PneumaticsConstants.kPCMPort, PneumaticsModuleType.CTREPCM, ArmConstants.kPistonForwardID, ArmConstants.kPistonReverseID);
        limitSwitch = new DigitalInput(ArmConstants.kLimitSwitchID);
        
        // gear ratio 27:1
        rotatingArm = new TalonFX(ArmConstants.kRotatingArmID);
        rotatingArm.setNeutralMode(NeutralMode.Brake);
        // CommandScheduler.getInstance().registerSubsystem(this);

        rotatingArm.setInverted(false);
        
        atTargetPosition = () -> NerdyMath.inRange(rotatingArm.getSelectedSensorPosition(), targetTicks - 1500, targetTicks + 1500);
        SmartDashboard.putNumber("Arm kP", ArmConstants.kArmP);
        SmartDashboard.putNumber("Arm kI", ArmConstants.kArmI);
        SmartDashboard.putNumber("Arm kD", ArmConstants.kArmD);
        SmartDashboard.putNumber("Arm kF", ArmConstants.kArmF);

        SmartDashboard.putNumber("Arm Cruise Velocity", ArmConstants.kArmCruiseVelocity);
        SmartDashboard.putNumber("Arm Accel", ArmConstants.kArmMotionAcceleration);
}

    public void moveArmJoystick(double currentJoystickOutput, double percentExtended) {
        // double armTicks = currentPosition.getAsDouble();
        
        if (currentJoystickOutput > ArmConstants.kArmDeadband) {
            
            rotatingArm.set(ControlMode.PercentOutput, 0.60);
            //((currentJoystickOutput * ArmConstants.kJoystickMultiplier)));
        } else if (currentJoystickOutput < -ArmConstants.kArmDeadband) {
            if (limitSwitch.get()) {
                rotatingArm.set(ControlMode.PercentOutput, 0);
                
            } else {
                rotatingArm.set(ControlMode.PercentOutput, -0.60);
            }
            // rotatingArm.setNeutralMode(NeutralMode.Coast);
                //((currentJoystickOutput * ArmConstants.kJoystickMultiplier)));
        } else {
            rotatingArm.set(ControlMode.PercentOutput, 0);
            rotatingArm.setNeutralMode(NeutralMode.Brake);
        }
        SmartDashboard.putNumber("Arm Joystick Input", currentJoystickOutput);

    }

    public void moveArmMotionMagicJoystick(double joystickInput, double perentExtended) {
        targetTicks += joystickInput * ArmConstants.kArmCruiseVelocity / 5;
        
        if (targetTicks < ArmConstants.kArmStow) {
            targetTicks = ArmConstants.kArmStow;
        }

        moveArmMotionMagic(targetTicks, perentExtended);
    }

    public CommandBase moveArmJoystickCommand(Supplier<Double> joystickInput) {
        return Commands.run(
            () -> moveArmJoystickCommand(joystickInput), this);
    }

    public void moveArmMotionMagic(int position, double percentExtended) {
        
        rotatingArm.config_kP(0, SmartDashboard.getNumber("Arm kP", ArmConstants.kArmP));
        rotatingArm.config_kI(0, SmartDashboard.getNumber("Arm kI", ArmConstants.kArmI));
        rotatingArm.config_kD(0, SmartDashboard.getNumber("Arm kD", ArmConstants.kArmD));
        rotatingArm.config_kF(0, SmartDashboard.getNumber("Arm kF", ArmConstants.kArmF));

        rotatingArm.configMotionCruiseVelocity(SmartDashboard.getNumber("Arm Cruise Velocity", ArmConstants.kArmCruiseVelocity));
        rotatingArm.configMotionAcceleration(SmartDashboard.getNumber("Arm Accel", ArmConstants.kArmMotionAcceleration));
        // config tuning params in slot 0
        double ff = -(ArmConstants.kStowedFF + ArmConstants.kDiffFF * percentExtended) * Math.cos(getArmAngle());
        rotatingArm.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, ff);
        targetTicks = position;

        SmartDashboard.putNumber("FF", ff);

        SmartDashboard.putBoolean("motion magic :(", true);

        // if (Math.abs(currentPosition.getAsDouble() - position) > 10) {
        //     rotatingArm.setNeutralMode(NeutralMode.Brake);
        // } else {
        //     rotatingArm.setNeutralMode(NeutralMode.Coast);
        // }
    }

    public void moveArmMotionMagic(double percentExtended) {
        
        rotatingArm.config_kP(0, SmartDashboard.getNumber("Arm kP", ArmConstants.kArmP));
        rotatingArm.config_kI(0, SmartDashboard.getNumber("Arm kI", ArmConstants.kArmI));
        rotatingArm.config_kD(0, SmartDashboard.getNumber("Arm kD", ArmConstants.kArmD));
        rotatingArm.config_kF(0, SmartDashboard.getNumber("Arm kF", ArmConstants.kArmF));

        rotatingArm.configMotionCruiseVelocity(SmartDashboard.getNumber("Arm Cruise Velocity", ArmConstants.kArmCruiseVelocity));
        rotatingArm.configMotionAcceleration(SmartDashboard.getNumber("Arm Accel", ArmConstants.kArmMotionAcceleration));
        // config tuning params in slot 0
        double ff = -(ArmConstants.kStowedFF + ArmConstants.kDiffFF * percentExtended) * Math.cos(getArmAngle());
        rotatingArm.set(ControlMode.MotionMagic, targetTicks, DemandType.ArbitraryFeedForward, ff);

        SmartDashboard.putNumber("FF", ff);

        SmartDashboard.putBoolean("motion magic :(", true);

        // if (Math.abs(currentPosition.getAsDouble() - position) > 10) {
        //     rotatingArm.setNeutralMode(NeutralMode.Brake);
        // } else {
        //     rotatingArm.setNeutralMode(NeutralMode.Coast);
        // }
    }

    public void setTargetTicks(int targetTicks) {
        this.targetTicks = targetTicks;
    }

    public void setPowerZero() {
        rotatingArm.set(ControlMode.PercentOutput, 0.0);
    }

    public void setBrakeMode() {
        rotatingArm.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {}

    public CommandBase moveArmScore(double percentExtended) {
        return Commands.run(
            () -> moveArmMotionMagic(ArmConstants.kArmScore, percentExtended), this
            
        );
    }

    public CommandBase moveArmGround(double percentExtended) {
        return Commands.run(
            () -> moveArmMotionMagic(ArmConstants.kArmGround, percentExtended), this
            
        );
    }

    public CommandBase moveArmStow(double percentExtended) {
        return Commands.run(
            () -> moveArmMotionMagic(ArmConstants.kArmStow, percentExtended), this
   
        );

    }

    public CommandBase moveArmPickUp() {
        return Commands.run(
            () -> moveArmMotionMagic(ArmConstants.kArmPickUp), this
            
        );
    }

    public double getArmAngle() {
        return Math.toRadians(Math.abs(rotatingArm.getSelectedSensorPosition()) / ArmConstants.kTicksPerAngle);
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

    public void resetEncoderStow() {
        rotatingArm.setSelectedSensorPosition(ArmConstants.kArmStow);
    }
    
    public void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Arm");

        tab.addNumber("Arm Motor Output", rotatingArm::getMotorOutputPercent);
        tab.addNumber("Arm Angle", () -> (ArmConstants.kArmStow * 2 - rotatingArm.getSelectedSensorPosition()) / ArmConstants.kTicksPerAngle);
        tab.addString("Arm Control Mode", rotatingArm.getControlMode()::toString);
        tab.addNumber("arm target velocity", rotatingArm::getActiveTrajectoryVelocity);
        tab.addNumber("arm velocity", rotatingArm::getSelectedSensorVelocity);
        // tab.addNumber("arm target velocity", rotatingArm::getActiveTrajectoryVelocity);
        tab.addNumber("Closed loop error", rotatingArm::getClosedLoopError);
        
        tab.addBoolean("Arm Extended", () -> armExtended);
        tab.addNumber("Current Arm Ticks", () -> rotatingArm.getSelectedSensorPosition());
        tab.addNumber("Target Arm Ticks", () -> targetTicks);
    }

    public void reportToSmartDashboard() {
        SmartDashboard.putBoolean("Limit switch", limitSwitch.get());

        SmartDashboard.putNumber("Arm Motor Output", rotatingArm.getMotorOutputPercent());
        SmartDashboard.putNumber("Arm Angle", Math.toDegrees(getArmAngle()));

        SmartDashboard.putString("Arm Control Mode", rotatingArm.getControlMode().toString());
        // SmartDashboard.putNumber("Closed Loop Target", rotatingArm.getClosedLoopTarget());
        SmartDashboard.putNumber("arm target velocity", rotatingArm.getActiveTrajectoryVelocity());
        SmartDashboard.putNumber("arm velocity", rotatingArm.getSelectedSensorVelocity());
        SmartDashboard.putNumber("arm target velocity", rotatingArm.getActiveTrajectoryVelocity());
        SmartDashboard.putNumber("Closed loop error", rotatingArm.getClosedLoopError());

        SmartDashboard.putBoolean("Arm Extended", armExtended);
        SmartDashboard.putNumber("Arm Ticks", rotatingArm.getSelectedSensorPosition());
        SmartDashboard.putNumber("Target Arm Ticks", targetTicks);
        if (this.getCurrentCommand() != null) {
            SmartDashboard.putBoolean("Arm subsystem", this.getCurrentCommand() == this.getDefaultCommand());
        }
    }
}

  

