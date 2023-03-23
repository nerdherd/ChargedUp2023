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
    // private DigitalInput talonTachTop;
    private boolean inTalonTachZone;
    // private DigitalInput talonTachBottom;

    public Arm() {
        // talonTachTop = new DigitalInput(ArmConstants.kTalonTachTopID);
        // talonTachBottom = new DigitalInput(ArmConstants.kTalonTachBottomID);
        
        // gear ratio 27:1
        rotatingArm = new TalonFX(ArmConstants.kRotatingArmID);
        rotatingArm.setNeutralMode(NeutralMode.Brake);
        // CommandScheduler.getInstance().registerSubsystem(this);

        rotatingArm.setInverted(false);
        
        atTargetPosition = () -> NerdyMath.inRange(rotatingArm.getSelectedSensorPosition(), targetTicks - 1500, targetTicks + 1500);
        
        // For tuning PID and Motion Magic
        // SmartDashboard.putNumber("Arm kP", ArmConstants.kArmP);
        // SmartDashboard.putNumber("Arm kI", ArmConstants.kArmI);
        // SmartDashboard.putNumber("Arm kD", ArmConstants.kArmD);
        // SmartDashboard.putNumber("Arm kF", ArmConstants.kArmF);

        // SmartDashboard.putNumber("Arm Cruise Velocity", ArmConstants.kArmCruiseVelocity);
        // SmartDashboard.putNumber("Arm Accel", ArmConstants.kArmMotionAcceleration);

        rotatingArm.config_kP(0, ArmConstants.kArmP);
        rotatingArm.config_kI(0, ArmConstants.kArmI);
        rotatingArm.config_kD(0, ArmConstants.kArmD);
        rotatingArm.config_kF(0, ArmConstants.kArmF);

        rotatingArm.configMotionCruiseVelocity(ArmConstants.kArmCruiseVelocity);
        rotatingArm.configMotionAcceleration(ArmConstants.kArmMotionAcceleration);
}

    public void moveArmJoystick(double currentJoystickOutput, double percentExtended) {
        // double armTicks = currentPosition.getAsDouble();
        
        if (currentJoystickOutput > ArmConstants.kArmDeadband) {
            
            // if (talonTachBottom.get() || rotatingArm.getStatorCurrent() >= 45) {
            if (rotatingArm.getStatorCurrent() >= 45)
            {
                rotatingArm.set(ControlMode.PercentOutput, 0);
            } else {
                rotatingArm.set(ControlMode.PercentOutput, 0.3);
            }

            // rotatingArm.set(ControlMode.PercentOutput, 0.60);
            //((currentJoystickOutput * ArmConstants.kJoystickMultiplier)));
        } else if (currentJoystickOutput < -ArmConstants.kArmDeadband) { // Up
            if (rotatingArm.getStatorCurrent() > 70) {
                rotatingArm.set(ControlMode.PercentOutput, 0);
            // if (talonTachTop.get() && rotatingArm.getStatorCurrent() >= 7) 
            // {
            //     rotatingArm.set(ControlMode.PercentOutput, 0);
            // } else if (talonTachTop.get()) {
            //     rotatingArm.set(ControlMode.PercentOutput, -0.1);
            } else {
                rotatingArm.set(ControlMode.PercentOutput, -0.3);
            }
            // rotatingArm.setNeutralMode(NeutralMode.Coast);
                //((currentJoystickOutput * ArmConstants.kJoystickMultiplier)));
        } else {
            rotatingArm.set(ControlMode.PercentOutput, 0);
            rotatingArm.setNeutralMode(NeutralMode.Brake);
        }
        // SmartDashboard.putNumber("Arm Joystick Input", currentJoystickOutput);

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
        setTargetTicks(position);
        moveArmMotionMagic(percentExtended);
    }

    public void moveArmMotionMagic(double percentExtended) {
        
        // rotatingArm.config_kP(0, SmartDashboard.getNumber("Arm kP", ArmConstants.kArmP));
        // rotatingArm.config_kI(0, SmartDashboard.getNumber("Arm kI", ArmConstants.kArmI));
        // rotatingArm.config_kD(0, SmartDashboard.getNumber("Arm kD", ArmConstants.kArmD));
        // rotatingArm.config_kF(0, SmartDashboard.getNumber("Arm kF", ArmConstants.kArmF));

        // rotatingArm.configMotionCruiseVelocity(SmartDashboard.getNumber("Arm Cruise Velocity", ArmConstants.kArmCruiseVelocity));
        // rotatingArm.configMotionAcceleration(SmartDashboard.getNumber("Arm Accel", ArmConstants.kArmMotionAcceleration));
        // config tuning params in slot 0
        double ff = -(ArmConstants.kStowedFF + ArmConstants.kDiffFF * percentExtended) * Math.cos(getArmAngle());

        // if (talonTachTop.get() && !inTalonTachZone) {
        //     if (rotatingArm.getSelectedSensorPosition() > ArmConstants.kArmTalonTach)
        //     {
        //         rotatingArm.setSelectedSensorPosition(ArmConstants.kArmTalonTach);
        //     }
        //     inTalonTachZone = true;
        
        // } else if (talonTachTop.get() && rotatingArm.getSelectedSensorPosition() > ArmConstants.kArmTalonTach) {
        //     rotatingArm.setSelectedSensorPosition(ArmConstants.kArmTalonTach);

        // } else if (!talonTachTop.get()){
        //     inTalonTachZone = false;
            
        // }
        

        if (targetTicks <= ArmConstants.kArmStow) {
            targetTicks = ArmConstants.kArmStow;
        }
        
        if (rotatingArm.getStatorCurrent() >= 70 && rotatingArm.getSelectedSensorPosition() > targetTicks)
        {
            rotatingArm.set(ControlMode.PercentOutput, 0);
        } else 
        {
            rotatingArm.set(ControlMode.MotionMagic, targetTicks, DemandType.ArbitraryFeedForward, ff);
            
        }
        // SmartDashboard.putNumber("Arm FF", ff);

        // SmartDashboard.putBoolean("arm motion magic :(", true);

        // if (Math.abs(currentPosition.getAsDouble() - position) > 10) {
        //     rotatingArm.setNeutralMode(NeutralMode.Brake);
        // } else {
        //     rotatingArm.setNeutralMode(NeutralMode.Coast);
        // }
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

    public CommandBase moveArm(int ticks, double percentExtended) {
        return Commands.run(
            () -> moveArmMotionMagic(ticks, percentExtended), this
        );
    }

    public CommandBase moveArm(int ticks, Supplier<Double> percentExtendedSupplier) {
        return Commands.run(
            () -> moveArmMotionMagic(ticks, percentExtendedSupplier.get()), this
        );
    }

    public CommandBase moveArm(Supplier<Double> percentExtendedSupplier) {
        return Commands.run(
            () -> moveArmMotionMagic(percentExtendedSupplier.get()), this
        );
    }

    public CommandBase moveArmScore(double percentExtended) {
        return Commands.run(
            () -> moveArmMotionMagic(ArmConstants.kArmScore, percentExtended), this
            
        );
    }

    public CommandBase moveArmScore(Supplier<Double> percentExtendedSupplier) {
        return moveArm(ArmConstants.kArmScore, percentExtendedSupplier);
    }

    public CommandBase moveArmGround(double percentExtended) {
        return Commands.run(
            () -> moveArmMotionMagic(ArmConstants.kArmGroundPickup, percentExtended), this
            
        );
    }

    public CommandBase moveArmGround(Supplier<Double> percentExtendedSupplier) {
        return moveArm(ArmConstants.kArmGroundPickup, percentExtendedSupplier);
    }

    public CommandBase moveArmStow(double percentExtended) {
        return Commands.run(
            () -> moveArmMotionMagic(ArmConstants.kArmStow, percentExtended), this
   
        );

    }

    public CommandBase moveArmStow(Supplier<Double> percentExtendedSupplier) {
        return moveArm(ArmConstants.kArmStow, percentExtendedSupplier);
    }

    public CommandBase moveArmPickUp(double percentExtended) {
        return Commands.run(
            () -> moveArmMotionMagic(ArmConstants.kArmSubstation, percentExtended), this
            
        );
    }

    public CommandBase moveArmPickup(Supplier<Double> percentExtendedSupplier) {
        return moveArm(ArmConstants.kArmSubstation, percentExtendedSupplier);
    }

    public double getArmAngle() {
        return Math.toRadians(Math.abs(rotatingArm.getSelectedSensorPosition()) / ArmConstants.kTicksPerAngle);
    }

    public void resetEncoderStow() {
        rotatingArm.setSelectedSensorPosition(ArmConstants.kArmStow);
    }

    
    public void armResetEncoder(int ticks) {
        rotatingArm.setSelectedSensorPosition(ticks);
    }

    public void isInTalonTachZone() {
        inTalonTachZone = true;
    }

    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF)  {
            return;
        }
        ShuffleboardTab tab;
        if (level == LOG_LEVEL.MINIMAL) {
            tab = Shuffleboard.getTab("Main");
        } else {
            tab = Shuffleboard.getTab("Arm");
        }
        switch (level) {
            case OFF:
                break;
            case ALL:
                tab.addNumber("Motor Output", rotatingArm::getMotorOutputPercent);
                tab.addString("Control Mode", rotatingArm.getControlMode()::toString);
                tab.addNumber("target velocity", rotatingArm::getActiveTrajectoryVelocity);
                tab.addNumber("arm target velocity", rotatingArm::getActiveTrajectoryVelocity);
                tab.addNumber("Closed loop error", rotatingArm::getClosedLoopError);
                tab.addNumber("Arm Current", rotatingArm::getStatorCurrent);
            case MEDIUM:
                tab.addNumber("Arm Velocity", rotatingArm::getSelectedSensorVelocity);
                tab.addNumber("Arm Voltage", rotatingArm::getMotorOutputVoltage);
                tab.addNumber("Arm Percent Output", rotatingArm::getMotorOutputPercent);
                tab.addNumber("Angle", () -> (ArmConstants.kArmStow * 2 - rotatingArm.getSelectedSensorPosition()) / ArmConstants.kTicksPerAngle);
            case MINIMAL:
                tab.addNumber("Current Arm Ticks", rotatingArm::getSelectedSensorPosition);
                tab.addNumber("Target Arm Ticks", () -> targetTicks);
                break;
        }

        
    }

    public void reportToSmartDashboard(LOG_LEVEL level) {
        switch (level) {
            case OFF:
                break;
            case ALL:
                // SmartDashboard.putBoolean("Limit switch", limitSwitch.get());
        
                SmartDashboard.putNumber("Arm Motor Output", rotatingArm.getMotorOutputPercent());
                SmartDashboard.putNumber("Arm Angle", Math.toDegrees(getArmAngle()));
        
                // SmartDashboard.putString("Arm Control Mode", rotatingArm.getControlMode().toString());
                // SmartDashboard.putNumber("Closed Loop Target", rotatingArm.getClosedLoopTarget());
                // SmartDashboard.putNumber("arm target velocity", rotatingArm.getActiveTrajectoryVelocity());
                SmartDashboard.putNumber("arm velocity", rotatingArm.getSelectedSensorVelocity());
                // SmartDashboard.putNumber("Closed loop error", rotatingArm.getClosedLoopError());
                // if (this.getCurrentCommand() != null) {
                //     SmartDashboard.putBoolean("Arm subsystem", this.getCurrentCommand() == this.getDefaultCommand());
                // }
            case MEDIUM:
                SmartDashboard.putNumber("Arm Current", rotatingArm.getStatorCurrent());
                SmartDashboard.putNumber("Arm Voltage", rotatingArm.getMotorOutputVoltage());
            case MINIMAL:
                SmartDashboard.putNumber("Arm Ticks", rotatingArm.getSelectedSensorPosition());
                SmartDashboard.putNumber("Target Arm Ticks", targetTicks);
                break;
        }


    }

}

  

