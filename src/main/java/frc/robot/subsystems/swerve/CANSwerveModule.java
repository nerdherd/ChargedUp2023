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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    private double CANCoderOffsetDegrees;

    private double currentPercent = 0;
    private double currentTurnPercent = 0;
    private double currentAngle = 0;
    private double desiredAngle = 0;
    private double desiredVelocity = 0;
    private boolean velocityControl = false;

    private SwerveModuleState desiredState = null;
    private SwerveModulePosition currPosition = new SwerveModulePosition();
    private SwerveModuleState currState = new SwerveModuleState();

    /**
     * Construct a new CANCoder Swerve Module.
     * 
     * @param driveMotorId
     * @param turningMotorId
     * @param invertDriveMotor
     * @param invertTurningMotor
     * @param CANCoderId
     * @param CANCoderOffsetDegrees
     * @param CANCoderReversed
     */
    public CANSwerveModule(int driveMotorId, int turningMotorId, boolean invertDriveMotor, boolean invertTurningMotor, 
    int CANCoderId, double CANCoderOffsetDegrees, boolean CANCoderReversed) {
        this.driveMotor = new TalonFX(driveMotorId, "CANivore2");
        this.turnMotor = new TalonFX(turningMotorId, "CANivore2");

        this.driveMotor.setNeutralMode(NeutralMode.Coast);
        this.turnMotor.setNeutralMode(NeutralMode.Coast);

        this.driveMotorID = driveMotorId;
        this.turnMotorID = turningMotorId;
        this.CANCoderID = CANCoderId;

        this.turningController = new PIDController(
            SmartDashboard.getNumber("kPTurning", ModuleConstants.kPTurning),
            SmartDashboard.getNumber("kITurning", ModuleConstants.kITurning),
            SmartDashboard.getNumber("kDTurning", ModuleConstants.kDTurning));
        turningController.enableContinuousInput(0, 2 * Math.PI); // Originally was -pi to pi
        turningController.setTolerance(.005);

        this.driveMotor.setInverted(invertDriveMotor);
        this.turnMotor.setInverted(invertTurningMotor);
        this.canCoder = new CANCoder(CANCoderId, "CANivore2");
        this.invertTurningEncoder = CANCoderReversed;
        this.CANCoderOffsetDegrees = CANCoderOffsetDegrees;
        
        this.desiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

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
     *          <td> Slot 0 </td> 
     *          <td> Relative Encoder </td>
     *      </tr>
     *      <tr>
     *          <td> CANCoder </td> 
     *          <td> Slot 1 </td> 
     *          <td> Absolute Encoder </td>
     *      </tr>
     *  </table>
     *  </table>
     */
    private void initEncoders() {
        driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1000);
        turnMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1000);

        driveMotor.configNeutralDeadband(ModuleConstants.kDriveMotorDeadband);
        turnMotor.configNeutralDeadband(ModuleConstants.kTurnMotorDeadband);
    }

    /**
     * Reset the CANCoder's relative encoder using its absolute encoder
     */
    public void resetEncoder() {
        double startAngle = (canCoder.getAbsolutePosition() - this.CANCoderOffsetDegrees) % 360;
        canCoder.setPosition(startAngle);
    }

    /**
     * Set the percent output of both motors to zero.
     */
    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        turnMotor.set(ControlMode.PercentOutput, 0);

        this.desiredState = new SwerveModuleState(0, Rotation2d.fromRadians(getTurningPosition()));
    }

    public void run() {
        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(getTurningPosition()));

        desiredAngle = desiredState.angle.getDegrees();

        double velocity = desiredState.speedMetersPerSecond / ModuleConstants.kDriveTicksPer100MsToMetersPerSec / ModuleConstants.kDriveMotorGearRatio;
        this.desiredVelocity = velocity;
        
        if (this.velocityControl) {
            driveMotor.config_kP(0, SmartDashboard.getNumber("kPDrive", ModuleConstants.kPDrive));
            driveMotor.config_kI(0, SmartDashboard.getNumber("kIDrive", ModuleConstants.kIDrive));
            driveMotor.config_kD(0, SmartDashboard.getNumber("kDDrive", ModuleConstants.kDDrive));
            driveMotor.config_kF(0, ModuleConstants.kFDrive);

            driveMotor.set(ControlMode.Velocity, velocity);
            this.currentPercent = 0;
        } else {
            this.currentPercent = desiredState.speedMetersPerSecond / SwerveDriveConstants.kPhysicalMaxSpeedMetersPerSecond;
            driveMotor.set(ControlMode.PercentOutput, this.currentPercent);
        }
        
        double turnPower = turningController.calculate(getTurningPosition(), desiredState.angle.getRadians());
        currentTurnPercent = turnPower;

        turnMotor.set(ControlMode.PercentOutput, turnPower);
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

    public double getDrivePositionTicks() {
        return driveMotor.getSelectedSensorPosition(0);
    }


    /**
     * Get the turning motor's CANCoder's angle
     * @return Angle in radians
     */
    public double getTurningPosition() {
        double turningPosition = Math.toRadians(getTurningPositionDegrees());
        return turningPosition;
    }

    /**
     * Get the turning motor's CANCoder's angle
     * @return Angle in degrees
     */
    public double getTurningPositionDegrees() {
        double turningPosition = canCoder.getPosition() % 360;
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
        double turnVelocity = Math.toRadians(getTurningVelocityDegrees());
        return turnVelocity;
    }

    /**
     * Get the velocity of the turning motor
     * @return Velocity of the turning motor (in radians / sec)
     */
    public double getTurningVelocityDegrees() {
        double turnVelocity = canCoder.getVelocity();
        return turnVelocity;
    }

    /**
     * Return the current state (velocity and rotation) of the Swerve Module
     * @return This Swerve Module's State
     */
    public SwerveModuleState getState() {
        currState.speedMetersPerSecond = getDriveVelocity();
        currState.angle = Rotation2d.fromRadians(getTurningPosition());
        return currState;
        // return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));

    }

    public SwerveModulePosition getPosition() {
        currPosition.distanceMeters = getDrivePosition();
        currPosition.angle = Rotation2d.fromRadians(getTurningPosition());
        return currPosition;
        //return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public double getTurnOffset() {
        return this.CANCoderOffsetDegrees;
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
            state.speedMetersPerSecond = 0;
        }

        this.desiredState = state;
    }

    public void setTurnOffset(double offset) {
        this.CANCoderOffsetDegrees = offset;
        resetEncoder();
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
                
                tab.addNumber("Turn Offset", () -> this.CANCoderOffsetDegrees);
                tab.addNumber("Turn percent (motor controller)", turnMotor::getMotorOutputPercent);
                tab.addNumber("Turn percent (current)", () -> this.currentTurnPercent);
            case MEDIUM:
                tab.addNumber("Drive Motor Current", driveMotor::getStatorCurrent);
                tab.addNumber("Turn Motor Current", turnMotor::getStatorCurrent);
                tab.addNumber("Drive Motor Voltage", driveMotor::getMotorOutputVoltage);
                tab.addNumber("Turn Motor Voltage", turnMotor::getMotorOutputVoltage);
                tab.addNumber("Module velocity", this::getDriveVelocity);
                tab.addNumber("Module velocity (ticks)", driveMotor::getSelectedSensorVelocity);
                tab.addNumber("Desired Velocity", () -> this.desiredVelocity);
                tab.addNumber("Drive percent (motor controller)", driveMotor::getMotorOutputPercent);
                tab.addNumber("Drive percent (current)", () -> this.currentPercent);
                
                tab.addNumber("Drive ticks", this::getDrivePositionTicks);
                tab.addNumber("Turn angle", this::getTurningPositionDegrees);
                tab.addNumber("Desired Angle", () -> desiredAngle);
                tab.addBoolean("Velocity Control", () -> this.velocityControl);
                tab.addNumber("Angle Difference", () -> desiredAngle - currentAngle);
                tab.addNumber("Drive Motor Bus Voltage", driveMotor::getBusVoltage);
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
                SmartDashboard.putNumber("Turn Offset", this.CANCoderOffsetDegrees);
            case MEDIUM:
                SmartDashboard.putNumber("Module velocity #" + driveMotorID, getDriveVelocity());
                SmartDashboard.putNumber("Drive percent #" + driveMotorID, driveMotor.getMotorOutputPercent());
                SmartDashboard.putNumber("Turn Angle #" + turnMotorID, currentAngle);
                SmartDashboard.putNumber("Turn Error #" + turnMotorID, desiredAngle - currentAngle);
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
