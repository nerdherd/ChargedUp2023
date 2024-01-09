package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveDriveConstants.CANCoderConstants;
import frc.robot.util.preferences.PrefDouble;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenixpro.configs.FeedbackConfigs;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

/**
 * Swerve module that uses CANCoder for the absolute position
 */
public class CANSwerveModule implements SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANCoder canCoder;
    private final TalonFXConfigurator driveConfigurator;
    private final TalonFXConfigurator turnConfigurator;

    private final DutyCycleOut driveRequest; 
    private final DutyCycleOut turnRequest;
    private final VelocityDutyCycle driveVelocityRequest;
    private final Slot0Configs drivePIDConfigs;

    private final int driveMotorID;
    private final int turnMotorID;
    private final int CANCoderID;

    private final PIDController turningController;
    private final boolean invertTurningEncoder;
    private PrefDouble CANCoderOffsetDegrees; 

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
    int CANCoderId, PrefDouble CANCoderOffsetDegrees, boolean CANCoderReversed) {
        this.driveMotor = new TalonFX(driveMotorId, ModuleConstants.kCANivoreName);
        this.turnMotor = new TalonFX(turningMotorId, ModuleConstants.kCANivoreName);
        
        this.driveConfigurator = driveMotor.getConfigurator();
        this.turnConfigurator = driveMotor.getConfigurator();
        
        this.driveRequest = new DutyCycleOut(0);
        this.turnRequest = new DutyCycleOut(0);
        this.driveRequest.EnableFOC = true;
        this.turnRequest.EnableFOC = true;
        
        this.driveVelocityRequest = new VelocityDutyCycle(0);
        this.driveVelocityRequest.EnableFOC = true;
        this.driveVelocityRequest.FeedForward = ModuleConstants.kFDrive;

        this.drivePIDConfigs = new Slot0Configs();
        this.driveConfigurator.refresh(drivePIDConfigs);
    

        MotorOutputConfigs driveConfigs = new MotorOutputConfigs();
        this.driveConfigurator.refresh(driveConfigs);
        driveConfigs.NeutralMode = NeutralModeValue.Coast;
        driveConfigs.DutyCycleNeutralDeadband =  ModuleConstants.kDriveMotorDeadband;
        this.driveConfigurator.apply(driveConfigs);

        MotorOutputConfigs turnConfigs = new MotorOutputConfigs();
        this.turnConfigurator.refresh(turnConfigs);
        turnConfigs.NeutralMode = NeutralModeValue.Coast;
        turnConfigs.DutyCycleNeutralDeadband =  ModuleConstants.kTurnMotorDeadband;
        this.turnConfigurator.apply(turnConfigs);


        this.driveMotorID = driveMotorId;
        this.turnMotorID = turningMotorId;
        this.CANCoderID = CANCoderId;

        this.turningController = new PIDController(
            ModuleConstants.kPTurning.get(),
            ModuleConstants.kITurning.get(),
            ModuleConstants.kDTurning.get());
        turningController.enableContinuousInput(0, 2 * Math.PI); // Originally was -pi to pi
        turningController.setTolerance(.005);

        this.driveMotor.setInverted(invertDriveMotor);
        this.turnMotor.setInverted(invertTurningMotor);
        this.canCoder = new CANCoder(CANCoderId, ModuleConstants.kCANivoreName);
        this.invertTurningEncoder = CANCoderReversed;
        this.CANCoderOffsetDegrees = CANCoderOffsetDegrees;
        
        this.desiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

        FeedbackConfigs driveFeedbackConfigs = new FeedbackConfigs();
        driveConfigurator.refresh(driveFeedbackConfigs);
        driveFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        driveConfigurator.apply(driveFeedbackConfigs);

        FeedbackConfigs turnFeedbackConfigs = new FeedbackConfigs();
        turnConfigurator.refresh(turnFeedbackConfigs);
        turnFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        turnConfigurator.apply(turnFeedbackConfigs);
    }

    /**
     * Reset the CANCoder's relative encoder using its absolute encoder
     */
    public void resetEncoder() {
        // double startAngle = (canCoder.getAbsolutePosition().getValue() * 360 - this.CANCoderOffsetDegrees.get()) % 360;
        // canCoder.setPosition(startAngle / 360);
        canCoder.setPosition(canCoder.getAbsolutePosition());

        ModuleConstants.ktunePID.loadPreferences();
        ModuleConstants.kPTurning.loadPreferences();
        ModuleConstants.kITurning.loadPreferences();
        ModuleConstants.kDTurning.loadPreferences();
        turningController.setPID(ModuleConstants.kPTurning.get(), ModuleConstants.kITurning.get(), ModuleConstants.kDTurning.get());
        
    }

    /**
     * Set the percent output of both motors to zero.
     */
    public void stop() {
        this.driveRequest.Output = 0;
        this.turnRequest.Output = 0;
        driveMotor.setControl(this.driveRequest);
        turnMotor.setControl(turnRequest);

        this.desiredState = new SwerveModuleState(0, Rotation2d.fromRadians(getTurningPosition()));
    }

    public void run() {
        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(getTurningPosition()));

        desiredAngle = desiredState.angle.getDegrees();

        double velocity = desiredState.speedMetersPerSecond / ModuleConstants.kDriveTicksPer100MsToMetersPerSec / ModuleConstants.kDriveMotorGearRatio;
        this.desiredVelocity = velocity;
        
        if (this.velocityControl) {
            if (ModuleConstants.ktunePID.get()) {
            
                this.drivePIDConfigs.kP = SmartDashboard.getNumber("kPDrive", ModuleConstants.kPDrive);
                this.drivePIDConfigs.kI = SmartDashboard.getNumber("kPDrive", ModuleConstants.kPDrive);
                this.drivePIDConfigs.kD = SmartDashboard.getNumber("kPDrive", ModuleConstants.kPDrive);

                driveConfigurator.apply(drivePIDConfigs);
            }

            driveMotor.setControl(driveVelocityRequest.withVelocity(velocity));
            this.currentPercent = 0;
        } else {
            this.currentPercent = desiredState.speedMetersPerSecond / SwerveDriveConstants.kPhysicalMaxSpeedMetersPerSecond;
            this.driveRequest.Output = currentPercent;
            driveMotor.setControl(this.driveRequest);
        
        }
        
        double turnPower = turningController.calculate(getTurningPosition(), desiredState.angle.getRadians());
        currentTurnPercent = turnPower;
        this.turnRequest.Output = currentTurnPercent;
        turnMotor.setControl(this.turnRequest);
    }

    public void flipModules() {
        this.CANCoderOffsetDegrees.set(this.CANCoderOffsetDegrees.get() + 180);
        this.CANCoderOffsetDegrees.uploadPreferences();
        //CANCoderConstants.kFLOffsetDeg.set(frontLeft.getTurnOffset() + frontLeft.getTurningPosition())
        resetEncoder();
    }

    public void resetDesiredAngle() {
        this.desiredAngle = 0;
    }
    
    //****************************** GETTERS ******************************/

    /**
     * Get the distance travelled by the motor in meters
     * @return Distance travelled by motor (in meters)
     */
    public double getDrivePosition() {
        return driveMotor.getRotorPosition().getValue()
            * ModuleConstants.kMetersPerRevolution
            * ModuleConstants.kDriveMotorGearRatio;
    }

    public double getDrivePositionTicks() {
        return driveMotor.getRotorPosition().getValue() * 2048;
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
        double turningPosition = (canCoder.getPosition()) % 360;
        return turningPosition;
    }

    /**
     * Get the velocity of the drive motor
     * @return Velocity of the drive motor (in meters / sec)
     */
    public double getDriveVelocity() {
        return driveMotor.getRotorVelocity().getValue() 
            * ModuleConstants.kMetersPerRevolution
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
     * @return Velocity of the turning motor (in degrees / sec)
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
        return this.CANCoderOffsetDegrees.get();
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
        this.CANCoderOffsetDegrees.set(offset);
        resetEncoder();
    }

    public void toggleVelocityControl(boolean velocityControlOn) {
        this.velocityControl = velocityControlOn;
    }

    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF)  {
            return;
        }
        int moduleId = (driveMotorID / 10);
        ShuffleboardTab tab = Shuffleboard.getTab("Module " + moduleId);

        switch (level) {
            case OFF:
                break;
            case ALL:
                
                tab.addNumber("Turn Offset", () -> this.CANCoderOffsetDegrees.get());
                tab.addNumber("Turn percent (motor controller)", () -> turnMotor.getDutyCycle().getValue());
                tab.addNumber("Turn percent (current)", () -> this.currentTurnPercent);
            case MEDIUM:
                tab.addNumber("Drive Motor Current", () -> driveMotor.getStatorCurrent().getValue());
                tab.addNumber("Turn Motor Current", () -> turnMotor.getStatorCurrent().getValue());
                tab.addNumber("Drive Motor Voltage", () -> (driveMotor.getDutyCycle().getValue() * driveMotor.getSupplyVoltage().getValue()));
                tab.addNumber("Turn Motor Voltage", () -> turnMotor.getSupplyVoltage().getValue());// ::getMotorOutputVoltage);
                tab.addNumber("Module velocity", this::getDriveVelocity);
                tab.addNumber("Desired Velocity", () -> this.desiredVelocity);
                tab.addNumber("Drive percent (motor controller)", () -> driveMotor.getDutyCycle().getValue());
                tab.addNumber("Drive percent (current)", () -> this.currentPercent);
                
                tab.addNumber("Drive ticks", this::getDrivePositionTicks);
                tab.addNumber("Turn angle", this::getTurningPositionDegrees);
                tab.addNumber("Turn angle percent", () -> turnMotor.getDutyCycle().getValue());
                tab.addNumber("Desired Angle", () -> desiredAngle);
                tab.addBoolean("Velocity Control", () -> this.velocityControl);
                tab.addNumber("Angle Difference", () -> desiredAngle - currentAngle);

                tab.add("Flip",Commands.runOnce(this::flipModules));
                // tab.addNumber("Drive Motor Bus Voltage", driveMotor::getBusVoltage);
            case MINIMAL:
                tab.addNumber("Turn angle", this::getTurningPositionDegrees);
                break;
        }

    }

     public void reportToSmartDashboard(LOG_LEVEL level) {
    //     currentAngle = Math.toDegrees(getTurningPosition());
    //     switch (level) {
    //         case OFF:
    //             break;
    //         case ALL:
    //             SmartDashboard.putNumber("Drive Motor #" + driveMotorID + " Current", driveMotor.getStatorCurrent().getValue());
    //             SmartDashboard.putNumber("Turn Motor #" + turnMotorID + " Current", turnMotor.getStatorCurrent().getValue());
    //             SmartDashboard.putNumber("Drive Motor #" + driveMotorID + " Voltage", (driveMotor.getDutyCycle().getValue() * driveMotor.getSupplyVoltage().getValue()));
    //             SmartDashboard.putNumber("Turn Motor #" + turnMotorID + " Voltage", (turnMotor.getDutyCycle().getValue() * turnMotor.getSupplyVoltage().getValue()));
    //             SmartDashboard.putNumber("Turn Offset", this.CANCoderOffsetDegrees.get());
    //         case MEDIUM:
    //             SmartDashboard.putNumber("Module velocity #" + driveMotorID, getDriveVelocity());
    //             SmartDashboard.putNumber("Drive percent #" + driveMotorID, driveMotor.getDutyCycle().getValue());
    //             SmartDashboard.putNumber("Turn Angle #" + turnMotorID, currentAngle);
    //             SmartDashboard.putNumber("Turn Error #" + turnMotorID, desiredAngle - currentAngle);
    //         case MINIMAL:
    //             break;
    //     }

     }

    /**
     * Enable or disable the break mode on the motors
     * @param breaking  Whether or not the motor should be on break mode
     */
    public void setBreak(boolean breaking) {
        NeutralModeValue mode = (breaking ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        
        MotorOutputConfigs turnConfigs = new MotorOutputConfigs();
        this.turnConfigurator.refresh(turnConfigs);
        turnConfigs.NeutralMode = mode;
        this.turnConfigurator.apply(turnConfigs);
        
        MotorOutputConfigs driveConfigs = new MotorOutputConfigs();
        this.driveConfigurator.refresh(driveConfigs);
        driveConfigs.NeutralMode = mode;
        this.driveConfigurator.apply(driveConfigs);
    }
}