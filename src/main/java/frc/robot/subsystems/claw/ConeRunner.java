package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConeRunnerConstants;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.Reportable.LOG_LEVEL;

public class ConeRunner extends SubsystemBase implements Reportable {
    private TalonSRX positionMotor;
    private TalonSRX speedMotor;
    private double targetTicks;

    public ConeRunner(){
        positionMotor = new TalonSRX(ConeRunnerConstants.kPositionID);
        speedMotor = new TalonSRX(ConeRunnerConstants.kSpeedID);
        positionMotor.setInverted(false);
        speedMotor.setInverted(false);
        positionMotor.setNeutralMode(NeutralMode.Brake);
        speedMotor.setNeutralMode(NeutralMode.Brake);

        positionMotor.configMotionAcceleration(
            ConeRunnerConstants.kConeRunnerMotionAcceleration);
        positionMotor.configMotionCruiseVelocity(
            ConeRunnerConstants.kConeRunnerCruiseVelocity);
        positionMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 1000);
    }

    public void resetEncoders() {
        positionMotor.setSelectedSensorPosition(0);
    }

    public void joystickSpeedControl(double power) {
        speedMotor.set(ControlMode.PercentOutput, power);
    }

    public void joystickAngleControl(double power) {
        positionMotor.set(ControlMode.PercentOutput, power);
    }

    public void angleMotionMagic(double position) {
        positionMotor.config_kP(0, SmartDashboard.getNumber("Cone Runner kP", ConeRunnerConstants.kConeRunnerP));
        positionMotor.config_kI(0, SmartDashboard.getNumber("Cone Runner kI", ConeRunnerConstants.kConeRunnerI));
        positionMotor.config_kD(0, SmartDashboard.getNumber("Cone Runner kD", ConeRunnerConstants.kConeRunnerD));
        positionMotor.config_kF(0, SmartDashboard.getNumber("Cone Runner kF", ConeRunnerConstants.kConeRunnerF));

        positionMotor.configMotionCruiseVelocity(
            SmartDashboard.getNumber(
                "Cone Runner Cruise Velocity", 
                ConeRunnerConstants.kConeRunnerCruiseVelocity));
        positionMotor.configMotionAcceleration(
            SmartDashboard.getNumber(
                "Arm Accel", 
                ConeRunnerConstants.kConeRunnerMotionAcceleration));
        // config tuning params in slot 0
        double angle = getAngle();
        double angleRadians = Math.toRadians(angle);
        double ff = -ConeRunnerConstants.kConeRunnerArbitraryFF * Math.cos(angleRadians);
        
        positionMotor.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, ff);
        targetTicks = position;
    }

    public double getAngle() {
        return ConeRunnerConstants.kInitialAngleDegrees 
            + (positionMotor.getSelectedSensorPosition(0) % 4096) 
            * ConeRunnerConstants.kDegreesPerTicks;
    }

    public void runConeRunner(){
        speedMotor.set(ControlMode.PercentOutput, 0.3); 
    }

    public void stopConeRunner(){
        speedMotor.set(ControlMode.PercentOutput, 0);
    }

    public void reportToSmartDashboard(LOG_LEVEL level) {
        switch (level) {
            case OFF:
                break;
            case ALL:
                SmartDashboard.putNumber("Cone Runner Intake Velocity", speedMotor.getSelectedSensorVelocity());
            case MEDIUM:
            case MINIMAL:
                SmartDashboard.putNumber("Cone Runner Target Ticks", targetTicks);   
                SmartDashboard.putNumber("Cone Runner Angle Ticks", positionMotor.getSelectedSensorPosition());
                break;
        }
    }

    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF)  {
            return;
        }
        ShuffleboardTab tab;
        if (level == LOG_LEVEL.MINIMAL) {
            tab = Shuffleboard.getTab("Main");
        } else {
            tab = Shuffleboard.getTab("Cone Runner");
        }

        switch (level) {
            case OFF:
                break;
            case ALL:
                tab.addNumber("Intake Velocity", speedMotor::getSelectedSensorVelocity);
            case MEDIUM:
            case MINIMAL:
                tab.addNumber("Cone Runner Target Ticks", () -> targetTicks);   
                tab.addNumber("Cone Runner Angle Ticks", positionMotor::getSelectedSensorPosition);
                break;
        }

        
    }
}
