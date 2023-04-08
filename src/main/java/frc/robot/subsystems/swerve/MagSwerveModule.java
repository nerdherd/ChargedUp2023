package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveDriveConstants;


/**
 * Swerve module that uses a Mag Encoder and SRX for the absolute angle
 */
public class MagSwerveModule implements SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    private final int driveMotorID;
    private final int turnMotorID;
    private final int absoluteEncoderId;

    private final PIDController turningController;
    private final TalonSRX absoluteTurningEncoder;
    private final boolean invertTurningEncoder;
    private final double absoluteEncoderOffset;

    private double currentAngle = 0;
    private double desiredAngle = 0;
    private double desiredVelocity = 0;
    private boolean velocityControl = false;

    /**
     * Constuct a new Swerve Module
     * 
     * @param driveMotorId              CAN ID of the drive motor
     * @param turningMotorId            CAN ID of the turning motor
     * @param invertDriveMotor          Whether or not the drive motor is inverted         
     * @param invertTurningMotor        Whether or not the turning motor is inverted
     * @param absoluteEncoderId         CAN ID of the absolute encoder
     * @param absoluteEncoderOffset     Zero position for the absolute encoder (in ticks)
     * @param absoluteEncoderReversed   Whether or not the absolute encoder is inverted
     */
    public MagSwerveModule(int driveMotorId, int turningMotorId, boolean invertDriveMotor, boolean invertTurningMotor, 
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.driveMotor = new TalonFX(driveMotorId);
        this.turnMotor = new TalonFX(turningMotorId);

        this.driveMotor.setNeutralMode(NeutralMode.Coast);
        this.turnMotor.setNeutralMode(NeutralMode.Coast);

        this.driveMotorID = driveMotorId;
        this.turnMotorID = turningMotorId;
        this.absoluteEncoderId = absoluteEncoderId;

        this.turningController = new PIDController(
            SmartDashboard.getNumber("kPTurning", ModuleConstants.kPTurning),
            SmartDashboard.getNumber("kITurning", ModuleConstants.kITurning),
            SmartDashboard.getNumber("kDTurning", ModuleConstants.kDTurning));
        turningController.enableContinuousInput(0, 2*Math.PI);
        turningController.setTolerance(.025);

        this.driveMotor.setInverted(invertDriveMotor);
        this.turnMotor.setInverted(invertTurningMotor);
        this.absoluteTurningEncoder = new TalonSRX(absoluteEncoderId);
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.invertTurningEncoder = absoluteEncoderReversed;

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
     *          <td> Talon SRX </td> 
     *          <td> PID 0 </td> 
     *          <td> Quadrature Encoder </td>
     *      </tr>
     *      <tr>
     *          <td> Talon SRX </td> 
     *          <td> PID 1 </td> 
     *          <td> PWM Mag Encoder (Absolute) </td>
     *      </tr>
     *  </table>
     */
    private void initEncoders() {
        driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1000);
        turnMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1000);
        absoluteTurningEncoder.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 1000);
        absoluteTurningEncoder.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.PulseWidthEncodedPosition, 1, 1000);
    }

    /**
     * Reset the SRX's quadrature encoder (slot 0) using the SRX's mag encoder (slot 1)
     */
    public void resetEncoder() {
        double startPos = (absoluteTurningEncoder.getSelectedSensorPosition(1) - absoluteEncoderOffset) % 4096;
        double startAngle = startPos / 4096;
        // SmartDashboard.putNumber("Reset Angle Encoder #" + absoluteEncoderId, startAngle);
        absoluteTurningEncoder.setSelectedSensorPosition(startPos, 0, 100);
    }

    /**
     * Set the percent output of both motors to zero.
     */
    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        // driveMotor.set(ControlMode.Velocity, 0);
        turnMotor.set(ControlMode.PercentOutput, 0);
        // setBreak(true);
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
     * Get the angle of the turning motor from the absolute encoder's quad encoder
     * @return Angle in radians
     */
    public double getTurningPosition() {
        double turningPosition = -(Math.IEEEremainder(absoluteTurningEncoder.getSelectedSensorPosition(0), 4096) * ModuleConstants.kAbsoluteTurningTicksToRad);
        // SmartDashboard.putNumber("turning position motor #" + turnMotorID, turningPosition);
        // SmartDashboard.putNumber("Turning angle #" + turnMotorID, 180 * turningPosition / Math.PI);
        return turningPosition;
    }

    /**
     * Get the velocity of the drive motor
     * @return Velocity of the drive motor (in meters / sec)
     */
    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity(0) 
            * ModuleConstants.kDriveTicksPer100MsToMetersPerSec
            * ModuleConstants.kDriveMotorGearRatio;
    }

    /**
     * Get the velocity of the turning motor
     * @return Velocity of the turning motor (in radians / sec)
     */
    public double getTurningVelocity() {
        double turnVelocity = turnMotor.getSelectedSensorVelocity(0) * ModuleConstants.kAbsoluteTurningTicksPer100MsToRadPerSec;
        // SmartDashboard.putNumber("Turn velocity Motor #" + turnMotorID, turnVelocity);
        return turnVelocity;
    }

    /**
     * Get the position of the absolute mag encoder
     * @return Position of the absolute mag encoder (in radians)
     */
    public double getAbsoluteEncoderRadians() {
        double angle = (absoluteTurningEncoder.getSelectedSensorPosition(1) % 4096) * ModuleConstants.kAbsoluteTurningTicksToRad + absoluteEncoderOffset;
        // SmartDashboard.putNumber("AbsoluteEncoder in rad", angle * (invertTurningEncoder ? -1 : 1));
        return angle * (invertTurningEncoder ? -1 : 1);
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
    public void setDesiredState(SwerveModuleState state, boolean withVelocityControl) {
        this.velocityControl = withVelocityControl;
        setDesiredState(state);
    }

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

        double velocity = state.speedMetersPerSecond / ModuleConstants.kDriveTicksPer100MsToMetersPerSec / ModuleConstants.kDriveMotorGearRatio;
        this.desiredVelocity = velocity;
        
        if (this.velocityControl) {
            driveMotor.config_kP(0, SmartDashboard.getNumber("kPDrive", ModuleConstants.kPDrive));
            driveMotor.config_kI(0, SmartDashboard.getNumber("kIDrive", ModuleConstants.kIDrive));
            driveMotor.config_kD(0, SmartDashboard.getNumber("kDDrive", ModuleConstants.kDDrive));
            // driveMotor.config_kP(0, ModuleConstants.kPDrive);
            // driveMotor.config_kI(0, ModuleConstants.kIDrive);
            // driveMotor.config_kD(0, ModuleConstants.kDDrive);
            driveMotor.config_kF(0, ModuleConstants.kFDrive);
            
            driveMotor.set(ControlMode.Velocity, velocity);
        } else {
            double currentPercent = state.speedMetersPerSecond / SwerveDriveConstants.kPhysicalMaxSpeedMetersPerSecond;
            driveMotor.set(ControlMode.PercentOutput, currentPercent);
        }
        double turnPower = turningController.calculate(getTurningPosition(), state.angle.getRadians());
        // SmartDashboard.putNumber("Turn Power Motor #" + turnMotorID, turnPower);

        turnMotor.set(ControlMode.PercentOutput, turnPower);
    }

    public void toggleVelocityControl(boolean velocityControlOn) {
        this.velocityControl = velocityControlOn;
    }

    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF || level == LOG_LEVEL.MINIMAL)  {
            return;
        }
        int moduleId = (driveMotorID / 10);
        ShuffleboardTab tab = Shuffleboard.getTab("Module " + moduleId);

        switch (level) {
            case OFF:
                break;
            case ALL:
                tab.addNumber("Drive Motor Current", driveMotor::getStatorCurrent);
                tab.addNumber("Turn Motor Current", turnMotor::getStatorCurrent);
            case MEDIUM:
                tab.addNumber("Drive Motor Voltage", driveMotor::getMotorOutputVoltage);
                tab.addNumber("Turn Motor Voltage", turnMotor::getMotorOutputVoltage);
                tab.addNumber("Module velocity", this::getDriveVelocity);
                tab.addNumber("Desired Velocity", () -> this.desiredVelocity);
                tab.addNumber("Drive percent", driveMotor::getMotorOutputPercent);
                tab.addNumber("Turn angle", () -> currentAngle);
                tab.addNumber("Desired Angle", () -> desiredAngle);
                tab.addBoolean("Velocity Control", () -> this.velocityControl);
                tab.addNumber("Angle Difference", () -> desiredAngle - currentAngle);
            case MINIMAL:
                break;
        }

    }

    public void reportToSmartDashboard(LOG_LEVEL level) {
        currentAngle = Math.toDegrees(getTurningPosition());
        switch (level) {
            case OFF:
                break;
            case ALL:
                SmartDashboard.putNumber("Drive Motor #" + driveMotorID + " Current", driveMotor.getStatorCurrent());
                SmartDashboard.putNumber("Turn Motor #" + turnMotorID + " Current", turnMotor.getStatorCurrent());
                SmartDashboard.putNumber("Drive Motor #" + driveMotorID + " Voltage", driveMotor.getMotorOutputVoltage());
                SmartDashboard.putNumber("Turn Motor #" + turnMotorID + " Voltage", turnMotor.getMotorOutputVoltage());
            case MEDIUM:
                SmartDashboard.putNumber("Module velocity #" + driveMotorID, getDriveVelocity());
                SmartDashboard.putNumber("Drive percent #" + driveMotorID, driveMotor.getMotorOutputPercent());
                SmartDashboard.putNumber("Turn angle #" + turnMotorID, currentAngle);
                SmartDashboard.putNumber("Desired Angle Motor #" + turnMotorID, desiredAngle);
                SmartDashboard.putNumber("Angle Difference Motor #" + turnMotorID, desiredAngle - currentAngle);
            case MINIMAL:
                break;
        }
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
