package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

/**
 * Swerve module that uses CANCoder for the absolute position
 */
public class CANSwerveModule implements SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANCoder canCoder;

    private final int driveMotorID;
    private final int turnMotorID;
    private final int CANCoderID;

    private final PIDController turningController;
    private final boolean invertTurningEncoder;
    private final double CANCoderOffset;

    private double currentAngle = 0;
    private double desiredAngle = 0;
    private double currentPercent = 0;

    /**
     * Constuct a new CANCoder Swerve Module
     * 
     * @param driveMotorId              CAN ID of the drive motor
     * @param turningMotorId            CAN ID of the turning motor
     * @param invertDriveMotor          Whether or not the drive motor is inverted         
     * @param invertTurningMotor        Whether or not the turning motor is inverted
     * @param absoluteEncoderId         CAN ID of the absolute encoder
     * @param absoluteEncoderOffset     Zero position for the absolute encoder (in ticks)
     * @param absoluteEncoderReversed   Whether or not the absolute encoder is inverted
     */
    public CANSwerveModule(int driveMotorId, int turningMotorId, boolean invertDriveMotor, boolean invertTurningMotor, 
    int CANCoderId, double absoluteEncoderOffset, boolean CANCoderReversed) {
        this.driveMotor = new TalonFX(driveMotorId);
        this.turnMotor = new TalonFX(turningMotorId);

        this.driveMotor.setNeutralMode(NeutralMode.Coast);
        this.turnMotor.setNeutralMode(NeutralMode.Coast);

        this.driveMotorID = driveMotorId;
        this.turnMotorID = turningMotorId;
        this.CANCoderID = CANCoderId;

        this.turningController = new PIDController(
            SmartDashboard.getNumber("kPTurning", ModuleConstants.kPTurning),
            SmartDashboard.getNumber("kITurning", ModuleConstants.kITurning),
            SmartDashboard.getNumber("kDTurning", ModuleConstants.kDTurning));
        turningController.enableContinuousInput(-Math.PI, Math.PI);
        turningController.setTolerance(.025);

        this.driveMotor.setInverted(invertDriveMotor);
        this.turnMotor.setInverted(invertTurningMotor);
        this.canCoder = new CANCoder(CANCoderId);
        this.CANCoderOffset = absoluteEncoderOffset;
        this.invertTurningEncoder = CANCoderReversed;

        initEncoders();
    }

    /**
     * Initialize the encoder type on each slot of the motor controllers
     * <p>
     * Note: avoid using PID 0 on the turning motor, 
     * as it is the same as PID 0 on the SRX.
     * <p>
     * Motor Encoders:
     *  <table>
     *      <tr>
     *          <td> Drive Motor </td> 
     *          <td> PID 0 </td> 
     *          <td> Integrated Sensor </td> 
     *      </tr>
     *      <tr>
     *          <td> Turn Motor </td> 
     *          <td> PID 0 </td> 
     *          <td> Integrated Sensor </td>
     *      </tr>
     *      <tr>
     *          <td> CANCoder </td> 
     *          <td> PID N/A </td> 
     *          <td> Mag Encoder </td>
     *      </tr>
     *  </table>
     */
    private void initEncoders() {
        driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1000);
        turnMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1000);
    }

    /**
     * Reset the turning motor's integrated sensor (slot 0) using the CANCoder
     */
    public void resetEncoder() {
        double startPos = (canCoder.getPosition() - CANCoderOffset) % 4096;
        double talonStartPos = startPos / 2;
        
        double startAngle = startPos / 4096;
        SmartDashboard.putNumber("Reset Angle Encoder #" + CANCoderID, startAngle);
        turnMotor.setSelectedSensorPosition(talonStartPos, 0, 100);
    }

    /**
     * Set the percent output of both motors to zero.
     */
    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        turnMotor.set(ControlMode.PercentOutput, 0);
    }

    //****************************** GETTERS ******************************/

    /**
     * Get the distance travelled by the motor in meters
     * @return Distance travelled by motor (in meters)
     */
    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition(0) 
            * ModuleConstants.kDriveTicksToMeters
            * ModuleConstants.kDriveMotorGearRatio;
    }

    /**
     * Get the angle of the turning motor's integrated sensor
     * @return Angle in radians
     */
    public double getTurningPosition() {
        double turningPosition = -(Math.IEEEremainder(turnMotor.getSelectedSensorPosition(0), 2048) * ModuleConstants.kAbsoluteTurningTicksToRad);
        // SmartDashboard.putNumber("turning position motor #" + turnMotorID, turningPosition);
        // SmartDashboard.putNumber("Turning angle #" + turnMotorID, 180 * turningPosition / Math.PI);
        return turningPosition;
    }

    /**
     * Get the velocity of the drive motor
     * @return Velocity of the drive motor (in meters / sec)
     */
    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity(0) * ModuleConstants.kDriveTicksPer100MsToMetersPerSec;
    }

    /**
     * Get the velocity of the turning motor
     * @return Velocity of the turning motor (in radians / sec)
     */
    public double getTurningVelocity() {
        double turnVelocity = turnMotor.getSelectedSensorVelocity(0) * ModuleConstants.kIntegratedTurningTicksPer100MsToRadPerSec;
        // SmartDashboard.putNumber("Turn velocity Motor #" + turnMotorID, turnVelocity);
        return turnVelocity;
    }

    /**
     * Return the current state (velocity and rotation) of the Swerve Module
     * @return This Swerve Module's State
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    //****************************** SETTERS ******************************/

    /**
     * Set the desired state of the Swerve Module and move towards it
     * @param state The desired state for this Swerve Module
     */
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        // state.angle = state.angle.rotateBy(Rotation2d.fromDegrees(-90));
        state = SwerveModuleState.optimize(state, getState().angle);

        desiredAngle = state.angle.getDegrees();
        
        // TODO: switch to velocity control
        // driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond);
        currentPercent = state.speedMetersPerSecond / SwerveDriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        driveMotor.set(ControlMode.PercentOutput, currentPercent);
        double turnPower = turningController.calculate(getTurningPosition(), state.angle.getRadians());
        // SmartDashboard.putNumber("Turn Power Motor #" + turnMotorID, turnPower);

        turnMotor.set(ControlMode.PercentOutput, turnPower);
    }

    public void reportToSmartDashboard() {
        currentAngle = Math.toDegrees(Math.toDegrees(getTurningPosition()));

        SmartDashboard.putNumber("Module velocity #" + driveMotorID, driveMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Drive percent #" + driveMotorID, currentPercent);
        SmartDashboard.putNumber("Turn angle #" + turnMotorID, currentAngle);
        SmartDashboard.putNumber("Desired Angle Motor #" + turnMotorID, desiredAngle);
        SmartDashboard.putNumber("Angle Difference Motor #" + turnMotorID, desiredAngle - currentAngle);
        SmartDashboard.putNumber("Current Motor #" + turnMotorID, driveMotor.getStatorCurrent());
    }

    /**
     * Enable or disable the break mode on the motors
     * @param breaking  Whether or not the motor should be on break mode
     */
    public void setBreak(boolean breaking) {
        NeutralMode mode = (breaking ? NeutralMode.Brake : NeutralMode.Coast);
        this.driveMotor.setNeutralMode(mode);
        this.turnMotor.setNeutralMode(mode);
    }
}
