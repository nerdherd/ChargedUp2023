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
    public DoubleSupplier armAngle;

    public Arm() {
        arm = new DoubleSolenoid(PneumaticsConstants.kPCMPort, PneumaticsModuleType.CTREPCM, ArmConstants.kPistonForwardID, ArmConstants.kPistonReverseID);
        
        // gear ratio 27:1
        rotatingArm = new TalonFX(ArmConstants.kRotatingArmID);

        // CommandScheduler.getInstance().registerSubsystem(this);
        initShuffleboard();

        rotatingArm.setInverted(false);
        rotatingArm.setSelectedSensorPosition(ArmConstants.kArmStow);
        

        rotatingArm.configMotionCruiseVelocity(ArmConstants.kArmCruiseVelocity);
        rotatingArm.configMotionAcceleration(ArmConstants.kArmMotionAcceleration);

        SmartDashboard.putNumber("Arm kP", ArmConstants.kArmP);
        SmartDashboard.putNumber("Arm kI", ArmConstants.kArmI);
        SmartDashboard.putNumber("Arm kD", ArmConstants.kArmD);
        SmartDashboard.putNumber("Arm kF", ArmConstants.kArmF);

        SmartDashboard.putNumber("Arm Cruise Velocity", ArmConstants.kArmCruiseVelocity);
        SmartDashboard.putNumber("Arm Accel", ArmConstants.kArmMotionAcceleration);


        atTargetPosition = () -> (NerdyMath.inRange(rotatingArm.getSelectedSensorPosition(), targetTicks - 1500, targetTicks + 1500));
        armAngle = () -> (Math.toRadians((ArmConstants.kArmStow * 2 - rotatingArm.getSelectedSensorPosition()) / ArmConstants.kTicksPerAngle));
    }

    public void moveArmJoystick(double currentJoystickOutput, double percentExtended) {
        // double armTicks = rotatingArm.getSelectedSensorPosition();
        
        if (currentJoystickOutput > ArmConstants.kArmDeadband) {
            if (rotatingArm.getSelectedSensorPosition() >= ArmConstants.kArmStow - 1500) {
                rotatingArm.set(ControlMode.PercentOutput, -(ArmConstants.kStowedFF + ArmConstants.kDiffFF * percentExtended) * Math.cos(armAngle.getAsDouble()));
            } else {
                rotatingArm.set(ControlMode.PercentOutput, 0.40);
                rotatingArm.setNeutralMode(NeutralMode.Coast);
            }
            //((currentJoystickOutput * ArmConstants.kJoystickMultiplier)));
        } else if (currentJoystickOutput < -ArmConstants.kArmDeadband) {
            if (rotatingArm.getSelectedSensorPosition() <= ArmConstants.kArmGround + 1500) {
                rotatingArm.set(ControlMode.PercentOutput, -(ArmConstants.kStowedFF + ArmConstants.kDiffFF * percentExtended) * Math.cos(armAngle.getAsDouble()));
            }
            rotatingArm.set(ControlMode.PercentOutput, -0.40);
            rotatingArm.setNeutralMode(NeutralMode.Coast);
                //((currentJoystickOutput * ArmConstants.kJoystickMultiplier)));
        } else {
            rotatingArm.set(ControlMode.PercentOutput, -(ArmConstants.kStowedFF + ArmConstants.kDiffFF * percentExtended) * Math.cos(armAngle.getAsDouble()));
            // rotatingArm.setNeutralMode(NeutralMode.Brake);
        }

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
        double angle = (ArmConstants.kArmStow * 2 - rotatingArm.getSelectedSensorPosition()) / ArmConstants.kTicksPerAngle;
        double angleRadians = Math.toRadians(angle);
        double ff = -(ArmConstants.kStowedFF + ArmConstants.kDiffFF * percentExtended) * Math.cos(angleRadians);
        rotatingArm.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, ff);
        targetTicks = position;

        SmartDashboard.putNumber("FF", ff);

        SmartDashboard.putBoolean("motion magic :(", true);

        // if (Math.abs(rotatingArm.getSelectedSensorPosition() - position) > 10) {
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
        double angle = (ArmConstants.kArmStow * 2 - rotatingArm.getSelectedSensorPosition()) / ArmConstants.kTicksPerAngle;
        double angleRadians = Math.toRadians(angle);
        double ff = -(ArmConstants.kStowedFF + ArmConstants.kDiffFF * percentExtended) * Math.cos(angleRadians);
        rotatingArm.set(ControlMode.MotionMagic, targetTicks, DemandType.ArbitraryFeedForward, ff);

        SmartDashboard.putNumber("FF", ff);

        SmartDashboard.putBoolean("motion magic :(", true);

        // if (Math.abs(rotatingArm.getSelectedSensorPosition() - position) > 10) {
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

    public CommandBase moveArmScore() {
        return Commands.run(
            () -> moveArmMotionMagic(ArmConstants.kArmScore), this
            
        );
    }

    public CommandBase moveArmGround() {
        return Commands.run(
            () -> moveArmMotionMagic(ArmConstants.kArmGround), this
            
        );
    }

    public CommandBase moveArmStow() {
        return Commands.run(
            () -> moveArmMotionMagic(ArmConstants.kArmStow), this
   
        );

    }

    public CommandBase moveArmPickUp() {
        return Commands.run(
            () -> moveArmMotionMagic(ArmConstants.kArmPickUp), this
            
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

    public void resetEncoder() {
        rotatingArm.setSelectedSensorPosition(ArmConstants.kArmStow);
    }
    
    private void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Arm");
        
        tab.addBoolean("Arm Extended", () -> armExtended);
        tab.addNumber("Current Arm Ticks", () -> rotatingArm.getSelectedSensorPosition());
        tab.addNumber("Target Arm Ticks", () -> targetTicks);
    }

    public void reportToSmartDashboard() {
        SmartDashboard.putNumber("Arm Motor Output", rotatingArm.getMotorOutputPercent());
        SmartDashboard.putNumber("Arm Angle", (ArmConstants.kArmStow * 2 - rotatingArm.getSelectedSensorPosition()) / ArmConstants.kTicksPerAngle);

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

  

