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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.util.NerdyMath;

public class Arm extends SubsystemBase implements Reportable {
    private DoubleSolenoid arm;
    private TalonFX rotatingArm;
    private TalonFX elevator;
    private boolean armExtended = false;
    private int armTargetTicks = ArmConstants.kArmStow;
    private int elevatorTargetTicks = ElevatorConstants.kElevatorStow;
    private PIDController armPID;
    public BooleanSupplier armAtTargetPosition;
    public BooleanSupplier elevatorAtTargetPosition;

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

        armAtTargetPosition = () -> (NerdyMath.inRange(rotatingArm.getSelectedSensorPosition(), armTargetTicks - 1500, armTargetTicks + 1500));
        elevatorAtTargetPosition = () -> (NerdyMath.inRange(elevator.getSelectedSensorPosition(), elevatorTargetTicks - 1500, elevatorTargetTicks + 1500));

        elevator = new TalonFX(ElevatorConstants.kElevatorID);
        elevator.setInverted(false);
        elevator.setSelectedSensorPosition(armTargetTicks);

        elevator.configMotionCruiseVelocity(ElevatorConstants.kElevatorCruiseVelocity);
        elevator.configMotionAcceleration(ElevatorConstants.kElevatorMotionAcceleration);

        SmartDashboard.putNumber("Elevator kP", ElevatorConstants.kElevatorP);
        SmartDashboard.putNumber("Elevator kI", ElevatorConstants.kElevatorI);
        SmartDashboard.putNumber("Elevator kD", ElevatorConstants.kElevatorD);
        SmartDashboard.putNumber("Elevator kF", ElevatorConstants.kElevatorF);

        SmartDashboard.putNumber("Elevator Cruise Velocity", ElevatorConstants.kElevatorCruiseVelocity);
        SmartDashboard.putNumber("Elevator Accel ", ElevatorConstants.kElevatorMotionAcceleration);


    }


    public void moveArmJoystick(double currentJoystickOutput) {
        // double armTicks = rotatingArm.getSelectedSensorPosition();

        double angle = (ArmConstants.kArmStow * 2 - rotatingArm.getSelectedSensorPosition()) / ArmConstants.kTicksPerAngle;
        double angleRadians = Math.toRadians(angle);
        double ff = -ArmConstants.kArbitraryFF * Math.cos(angleRadians);

        if (currentJoystickOutput >= 0.40 ) {
            currentJoystickOutput = 0.40;
        }

        // within range
        // if (rotatingArm.getSelectedSensorPosition() > ArmConstants.kArmScore) {

            if (currentJoystickOutput > ArmConstants.kArmDeadband) {
                // if (rotatingArm.getSelectedSensorPosition() > 354255) {

                    rotatingArm.set(ControlMode.PercentOutput, 0.70);
                // } else {
                //     rotatingArm.set(ControlMode.PercentOutput, 0);
                // }
                // rotatingArm.setNeutralMode(NeutralMode.Coast);
                //((currentJoystickOutput * ArmConstants.kJoystickMultiplier)));
            } else if (currentJoystickOutput < -ArmConstants.kArmDeadband) {
                if (rotatingArm.getSelectedSensorPosition() > 500000) {
                    rotatingArm.set(ControlMode.PercentOutput, -0.70);

                } else {
                    rotatingArm.set(ControlMode.PercentOutput, 0);
                }

                // rotatingArm.setNeutralMode(NeutralMode.Coast);
                    //((currentJoystickOutput * ArmConstants.kJoystickMultiplier)));
            } else {
                rotatingArm.set(ControlMode.PercentOutput, 0);
                rotatingArm.setNeutralMode(NeutralMode.Brake);
            }
    
        // } else { // going up past limit
        //     rotatingArm.set(ControlMode.PercentOutput, 0);
        //     rotatingArm.setNeutralMode(NeutralMode.Brake);

        // }
        
    }

    

    public CommandBase moveArmJoystickCommand(Supplier<Double> joystickInput) {
        return Commands.run(
            () -> moveArmJoystickCommand(joystickInput), this);
    }

    public void moveElevatorJoystick(double currentJoystickOutput) {
        if (NerdyMath.inRange(elevator.getSelectedSensorPosition(), ElevatorConstants.kElevatorUpperLimit - 1500, ElevatorConstants.kElevatorStow + 1500)) {
            if (currentJoystickOutput > ElevatorConstants.kElevatorDeadband) {
                elevator.set(ControlMode.PercentOutput, 0.40);
                elevator.setNeutralMode(NeutralMode.Coast);
            } else if (currentJoystickOutput < -ElevatorConstants.kElevatorDeadband) {
                elevator.set(ControlMode.PercentOutput, -0.40);
                elevator.setNeutralMode(NeutralMode.Coast);
            } else {
                elevator.set(ControlMode.PercentOutput, 0);
                elevator.setNeutralMode(NeutralMode.Brake);
            }
        }

    }

    public CommandBase moveElevatorJoystickCommand(Supplier<Double> joystickInput) {
        return Commands.run(
            () -> moveElevatorJoystickCommand(joystickInput), this);

    }

    public void moveArmMotionMagic(int position) {
        
        rotatingArm.config_kP(0, SmartDashboard.getNumber("Arm kP", ArmConstants.kArmP));
        rotatingArm.config_kI(0, SmartDashboard.getNumber("Arm kI", ArmConstants.kArmI));
        rotatingArm.config_kD(0, SmartDashboard.getNumber("Arm kD", ArmConstants.kArmD));
        rotatingArm.config_kF(0, SmartDashboard.getNumber("Arm kF", ArmConstants.kArmF));

        rotatingArm.configMotionCruiseVelocity(SmartDashboard.getNumber("Arm Cruise Velocity", ArmConstants.kArmCruiseVelocity));
        rotatingArm.configMotionAcceleration(SmartDashboard.getNumber("Arm Accel", ArmConstants.kArmMotionAcceleration));
        // config tuning params in slot 0
        double angle = (ArmConstants.kArmStow * 2 - rotatingArm.getSelectedSensorPosition()) / ArmConstants.kTicksPerAngle;
        double angleRadians = Math.toRadians(angle);
        double ff = -ArmConstants.kArbitraryFF * Math.cos(angleRadians);
        rotatingArm.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, ff);
        armTargetTicks = position;

        SmartDashboard.putNumber("Arm FF", ff);

        SmartDashboard.putBoolean("arm motion magic :(", true);

        // if (Math.abs(rotatingArm.getSelectedSensorPosition() - position) > 10) {
        //     rotatingArm.setNeutralMode(NeutralMode.Brake);
        // } else {
        //     rotatingArm.setNeutralMode(NeutralMode.Coast);
        // }
    }

    public void moveArmMotionMagic() {
        
        rotatingArm.config_kP(0, SmartDashboard.getNumber("Arm kP", ArmConstants.kArmP));
        rotatingArm.config_kI(0, SmartDashboard.getNumber("Arm kI", ArmConstants.kArmI));
        rotatingArm.config_kD(0, SmartDashboard.getNumber("Arm kD", ArmConstants.kArmD));
        rotatingArm.config_kF(0, SmartDashboard.getNumber("Arm kF", ArmConstants.kArmF));

        rotatingArm.configMotionCruiseVelocity(SmartDashboard.getNumber("Arm Cruise Velocity", ArmConstants.kArmCruiseVelocity));
        rotatingArm.configMotionAcceleration(SmartDashboard.getNumber("Arm Accel", ArmConstants.kArmMotionAcceleration));
        // config tuning params in slot 0
        double angle = (ArmConstants.kArmStow * 2 - rotatingArm.getSelectedSensorPosition()) / ArmConstants.kTicksPerAngle;
        double angleRadians = Math.toRadians(angle);
        double ff = -ArmConstants.kArbitraryFF * Math.cos(angleRadians);
        rotatingArm.set(ControlMode.MotionMagic, armTargetTicks, DemandType.ArbitraryFeedForward, ff);

        SmartDashboard.putNumber("Arm FF", ff);

        SmartDashboard.putBoolean("arm motion magic :(", true);

        // if (Math.abs(rotatingArm.getSelectedSensorPosition() - position) > 10) {
        //     rotatingArm.setNeutralMode(NeutralMode.Brake);
        // } else {
        //     rotatingArm.setNeutralMode(NeutralMode.Coast);
        // }
    }

    public void moveElevatorMotionMagic(int position) {
        elevator.config_kP(0, SmartDashboard.getNumber("Elevator kP", ElevatorConstants.kElevatorP));
        elevator.config_kI(0, SmartDashboard.getNumber("Elevator kI", ElevatorConstants.kElevatorI));
        elevator.config_kD(0, SmartDashboard.getNumber("Elevator kD", ElevatorConstants.kElevatorD));
        elevator.config_kF(0, SmartDashboard.getNumber("Elevator kF",ElevatorConstants.kElevatorF));

        elevator.configMotionCruiseVelocity(SmartDashboard.getNumber("Elevator Cruise Velocity", ElevatorConstants.kElevatorCruiseVelocity));
        elevator.configMotionAcceleration(SmartDashboard.getNumber("Elevator Motion Acceleration", ElevatorConstants.kElevatorMotionAcceleration));

        double angle = (ArmConstants.kArmStow * 2 - rotatingArm.getSelectedSensorPosition()) / ArmConstants.kTicksPerAngle;
        double angleRadians = Math.toRadians(angle);
        double ff = -ArmConstants.kArbitraryFF * Math.sin(angleRadians);
        elevator.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, ff);
        elevatorTargetTicks = position;

        SmartDashboard.putNumber("Elevator FF", ff);

        SmartDashboard.putBoolean("elevator motion magic :(", true);

    }

    public void moveElevatorMotionMagic() {
        
        elevator.config_kP(0, SmartDashboard.getNumber("Elevator kP", ElevatorConstants.kElevatorP));
        elevator.config_kI(0, SmartDashboard.getNumber("Elevator kI", ElevatorConstants.kElevatorI));
        elevator.config_kD(0, SmartDashboard.getNumber("Elevator kD", ElevatorConstants.kElevatorD));
        elevator.config_kF(0, SmartDashboard.getNumber("Elevator kF", ElevatorConstants.kElevatorF));

        elevator.configMotionCruiseVelocity(SmartDashboard.getNumber("Elevator Cruise Velocity", ElevatorConstants.kElevatorCruiseVelocity));
        elevator.configMotionAcceleration(SmartDashboard.getNumber("Elevator Accel", ElevatorConstants.kElevatorMotionAcceleration));
        // config tuning params in slot 0
        double angle = (ArmConstants.kArmStow * 2 - rotatingArm.getSelectedSensorPosition()) / ArmConstants.kTicksPerAngle;
        double angleRadians = Math.toRadians(angle);
        double ff = -ArmConstants.kArbitraryFF * Math.cos(angleRadians);
        elevator.set(ControlMode.MotionMagic, elevatorTargetTicks, DemandType.ArbitraryFeedForward, ff);

        SmartDashboard.putNumber("Elevator FF", ff);

        SmartDashboard.putBoolean("elevator motion magic :(", true);

    }

    public void setArmTargetTicks(int armTargetTicks) {
        this.armTargetTicks = armTargetTicks;
    }

    public void setElevatorTargetTicks(int elevatorTargetTicks) {
        this.elevatorTargetTicks = elevatorTargetTicks;
    }

    public void setArmPowerZero() {
        rotatingArm.set(ControlMode.PercentOutput, 0.0);
    }

    public void setArmBrakeMode() {
        rotatingArm.setNeutralMode(NeutralMode.Brake);
    }

    public void setElevatorPowerZero() {
        elevator.set(ControlMode.PercentOutput, 0.0);
    }

    public void setElevatorBrakeMode() {
        elevator.setNeutralMode(NeutralMode.Brake);
    }

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

    public CommandBase moveElevatorScoreMid() {
        return Commands.run(
            () -> moveArmMotionMagic(ElevatorConstants.kElevatorScoreMid), this
            
        );
    }

    public CommandBase moveElevatorScoreHigh() {
        return Commands.run(
            () -> moveArmMotionMagic(ElevatorConstants.kElevatorScoreHigh), this
            
        );
    }

    public CommandBase moveElevatorStow() {
        return Commands.run(
            () -> moveArmMotionMagic(ElevatorConstants.kElevatorStow), this
            
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

    public void armResetEncoder() {
        rotatingArm.setSelectedSensorPosition(ArmConstants.kArmStow);
    }

    
    public void armResetEncoder(int ticks) {
        rotatingArm.setSelectedSensorPosition(ticks);
    }

    public void elevatorResetEncoder() {
        elevator.setSelectedSensorPosition(ElevatorConstants.kElevatorStow);
    }
    
    private void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Arm");
        
        tab.addBoolean("Arm Extended", () -> armExtended);
        tab.addNumber("Current Arm Ticks", () -> rotatingArm.getSelectedSensorPosition());
        tab.addNumber("Target Arm Ticks", () -> armTargetTicks);
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
        SmartDashboard.putNumber("Target Arm Ticks", armTargetTicks);
        if (this.getCurrentCommand() != null) {
            SmartDashboard.putBoolean("Arm subsystem", this.getCurrentCommand() == this.getDefaultCommand());
        }

        SmartDashboard.putNumber("Elevator Motor Output", elevator.getMotorOutputPercent());

        SmartDashboard.putString("Elevator Control Mode", elevator.getControlMode().toString());
        SmartDashboard.putNumber("elevator target velocity", elevator.getActiveTrajectoryVelocity());
        SmartDashboard.putNumber("elevator velocity", elevator.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Closed loop error", elevator.getClosedLoopError());

        SmartDashboard.putNumber("Elevator Ticks", elevator.getSelectedSensorPosition());
        SmartDashboard.putNumber("Target Elevator Ticks", elevatorTargetTicks);
    }

}

  

