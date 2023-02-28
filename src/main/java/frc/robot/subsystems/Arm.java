// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.NerdyMath;

public class Arm extends SubsystemBase implements Reportable {
    private TalonFX rotatingArm;
    private int targetTicks = ArmConstants.kArmStow;
    public BooleanSupplier atTargetPosition;
    public DoubleSupplier percentExtended;
    public IntSupplier elevatorTicks;
    private DigitalInput limitSwitch;

    public Arm() {
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

    public void setPercentExtended(DoubleSupplier percentExtended) {
        this.percentExtended = percentExtended;
    }

    public double getPercentExtended() {
        return percentExtended.getAsDouble();
    }

    public void setElevatorTicks(IntSupplier elevatorTicks) {
        this.elevatorTicks = elevatorTicks;
    }

    public int getElevatorTicks() {
        return elevatorTicks.getAsInt();
    }

    public void moveArmJoystick(double currentJoystickOutput) {
        // double armTicks = currentPosition.getAsDouble();
        
        if (currentJoystickOutput > ArmConstants.kArmDeadband) {
            
            if (rotatingArm.getSelectedSensorPosition() >= ArmConstants.kArmLowerLimit) {
                rotatingArm.set(ControlMode.PercentOutput, 0);
              } else {
                rotatingArm.set(ControlMode.PercentOutput, 0.8);
            }

            // rotatingArm.set(ControlMode.PercentOutput, 0.60);
            //((currentJoystickOutput * ArmConstants.kJoystickMultiplier)));
        } else if (currentJoystickOutput < -ArmConstants.kArmDeadband) { // Up
            if (limitSwitch.get()) {
                rotatingArm.set(ControlMode.PercentOutput, 0);
                armResetEncoderStow();
            } else {
                rotatingArm.set(ControlMode.PercentOutput, -0.8);
            }
            // rotatingArm.setNeutralMode(NeutralMode.Coast);
                //((currentJoystickOutput * ArmConstants.kJoystickMultiplier)));
        } else {
            
            if (getElevatorTicks() * Math.sin(getArmAngle()) >= 77) {
                double ff = -(ArmConstants.kStowedFF + ArmConstants.kDiffFF * getPercentExtended()) * Math.cos(getArmAngle());
                targetTicks = (int) (Math.toDegrees(1 / Math.sin(77 / getElevatorTicks())) * ArmConstants.kTicksPerAngle);
                moveArmMotionMagic();
            } else {
                rotatingArm.set(ControlMode.PercentOutput, 0);
                rotatingArm.setNeutralMode(NeutralMode.Brake);
                
            }

        }
        SmartDashboard.putNumber("Arm Joystick Input", currentJoystickOutput);


    }

    public void moveArmMotionMagicJoystick(double joystickInput) {
        targetTicks += joystickInput * ArmConstants.kArmCruiseVelocity / 5;
        
        if (targetTicks < ArmConstants.kArmStow) {
            targetTicks = ArmConstants.kArmStow;
        }

        moveArmMotionMagic(targetTicks);
    }

    

    public CommandBase moveArmJoystickCommand(Supplier<Double> joystickInput) {
        return Commands.run(
            () -> moveArmJoystickCommand(joystickInput), this);
    }

    public CommandBase moveArmMotionMagic(int position) {
        
        rotatingArm.config_kP(0, SmartDashboard.getNumber("Arm kP", ArmConstants.kArmP));
        rotatingArm.config_kI(0, SmartDashboard.getNumber("Arm kI", ArmConstants.kArmI));
        rotatingArm.config_kD(0, SmartDashboard.getNumber("Arm kD", ArmConstants.kArmD));
        rotatingArm.config_kF(0, SmartDashboard.getNumber("Arm kF", ArmConstants.kArmF));

        rotatingArm.configMotionCruiseVelocity(SmartDashboard.getNumber("Arm Cruise Velocity", ArmConstants.kArmCruiseVelocity));
        rotatingArm.configMotionAcceleration(SmartDashboard.getNumber("Arm Accel", ArmConstants.kArmMotionAcceleration));
        // config tuning params in slot 0
        double ff = -(ArmConstants.kStowedFF + ArmConstants.kDiffFF * getPercentExtended()) * Math.cos(getArmAngle());
        targetTicks = position;

        SmartDashboard.putNumber("Arm FF", ff);

        SmartDashboard.putBoolean("arm motion magic :(", true);

        // if (Math.abs(currentPosition.getAsDouble() - position) > 10) {
        //     rotatingArm.setNeutralMode(NeutralMode.Brake);
        // } else {
        //     rotatingArm.setNeutralMode(NeutralMode.Coast);
        // }

        if (getElevatorTicks() * Math.sin(getArmAngle()) >= 77) {
            targetTicks = (int) (Math.toDegrees(1 / Math.sin(77 / getElevatorTicks())) * ArmConstants.kTicksPerAngle);
        }
        return Commands.run(() -> rotatingArm.set(ControlMode.MotionMagic, targetTicks, DemandType.ArbitraryFeedForward, ff));
        

    }

    public void moveArmMotionMagic() {
        
        rotatingArm.config_kP(0, SmartDashboard.getNumber("Arm kP", ArmConstants.kArmP));
        rotatingArm.config_kI(0, SmartDashboard.getNumber("Arm kI", ArmConstants.kArmI));
        rotatingArm.config_kD(0, SmartDashboard.getNumber("Arm kD", ArmConstants.kArmD));
        rotatingArm.config_kF(0, SmartDashboard.getNumber("Arm kF", ArmConstants.kArmF));

        rotatingArm.configMotionCruiseVelocity(SmartDashboard.getNumber("Arm Cruise Velocity", ArmConstants.kArmCruiseVelocity));
        rotatingArm.configMotionAcceleration(SmartDashboard.getNumber("Arm Accel", ArmConstants.kArmMotionAcceleration));
        // config tuning params in slot 0
        double ff = -(ArmConstants.kStowedFF + ArmConstants.kDiffFF * getPercentExtended()) * Math.cos(getArmAngle());

        SmartDashboard.putNumber("Arm FF", ff);

        SmartDashboard.putBoolean("arm motion magic :(", true);

        // if (Math.abs(currentPosition.getAsDouble() - position) > 10) {
        //     rotatingArm.setNeutralMode(NeutralMode.Brake);
        // } else {
        //     rotatingArm.setNeutralMode(NeutralMode.Coast);
        // }

        
        if (getElevatorTicks() * Math.sin(getArmAngle()) >= 77) {
            targetTicks = (int) (Math.toDegrees(1 / Math.sin(77 / getElevatorTicks())) * ArmConstants.kTicksPerAngle);
        }
        rotatingArm.set(ControlMode.MotionMagic, targetTicks, DemandType.ArbitraryFeedForward, ff);
        
    }

    public void setTargetTicks(int targetTicks) {
        this.targetTicks = targetTicks;
    }

    public void setArmPowerZero() {
        rotatingArm.set(ControlMode.PercentOutput, 0.0);
    }

    public void setArmBrakeMode() {
        rotatingArm.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {}


  
    public double getArmAngle() {
        return Math.toRadians(Math.abs(rotatingArm.getSelectedSensorPosition()) / ArmConstants.kTicksPerAngle);
    }

    public void armResetEncoderStow() {
        rotatingArm.setSelectedSensorPosition(ArmConstants.kArmStow);
    }

    
    public void armResetEncoder(int ticks) {
        rotatingArm.setSelectedSensorPosition(ticks);
    }

    public void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Arm");

        tab.addNumber("Motor Output", rotatingArm::getMotorOutputPercent);
        tab.addNumber("Angle", () -> (ArmConstants.kArmStow * 2 - rotatingArm.getSelectedSensorPosition()) / ArmConstants.kTicksPerAngle);
        tab.addString("Control Mode", rotatingArm.getControlMode()::toString);
        tab.addNumber("target velocity", rotatingArm::getActiveTrajectoryVelocity);
        tab.addNumber("velocity", rotatingArm::getSelectedSensorVelocity);
        // tab.addNumber("arm target velocity", rotatingArm::getActiveTrajectoryVelocity);
        tab.addNumber("Closed loop error", rotatingArm::getClosedLoopError);
        
        tab.addNumber("Current Arm Ticks", () -> rotatingArm.getSelectedSensorPosition());
        tab.addNumber("Target Arm Ticks", () -> targetTicks);
        tab.addNumber("Arm Current", rotatingArm::getStatorCurrent);
        tab.addNumber("Arm Voltage", rotatingArm::getMotorOutputVoltage);
        tab.addNumber("Arm FF", () -> -(ArmConstants.kStowedFF + ArmConstants.kDiffFF * getPercentExtended()) * Math.cos(getArmAngle()));

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

        SmartDashboard.putNumber("Arm Ticks", rotatingArm.getSelectedSensorPosition());
        SmartDashboard.putNumber("Target Arm Ticks", targetTicks);
        if (this.getCurrentCommand() != null) {
            SmartDashboard.putBoolean("Arm subsystem", this.getCurrentCommand() == this.getDefaultCommand());
        }

        SmartDashboard.putNumber("Arm Current", rotatingArm.getStatorCurrent());
        SmartDashboard.putNumber("Arm Voltage", rotatingArm.getMotorOutputVoltage());
    }

}

  

