package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveDriveConstants.CANCoderConstants;
import frc.robot.Constants.SwerveDriveConstants.MagEncoderConstants;
import frc.robot.subsystems.Imu;
import frc.robot.subsystems.Reportable;

import static frc.robot.Constants.SwerveDriveConstants.*;

public class SwerveDrivetrain extends SubsystemBase implements Reportable {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final Imu gyro;
    private final SwerveDriveOdometry odometer;

    private Field2d field;

    private int numEncoderResets = 0;

    public enum SwerveModuleType {
        MAG_ENCODER,
        CANCODER
    }

    /**
     * Construct a new {@link SwerveDrivetrain}
     */
    public SwerveDrivetrain(Imu gyro, SwerveModuleType moduleType) throws IllegalArgumentException {
        switch (moduleType) {
            case MAG_ENCODER:
                frontLeft = new MagSwerveModule(
                    kFLDriveID,
                    kFLTurningID,
                    kFLDriveReversed,
                    kFLTurningReversed,
                    MagEncoderConstants.kFLAbsoluteID,
                    MagEncoderConstants.kFLDefaultOffsetTicks,
                    MagEncoderConstants.kFLAbsoluteReversed,
                    kFLGearRatio,
                    kFLMaxSpeed);
                frontRight = new MagSwerveModule(
                    kFRDriveID,
                    kFRTurningID,
                    kFRDriveReversed,
                    kFRTurningReversed,
                    MagEncoderConstants.kFRAbsoluteID,
                    MagEncoderConstants.kFRDefaultOffsetTicks,
                    MagEncoderConstants.kFRAbsoluteReversed,
                    kFRGearRatio,
                    kFRMaxSpeed);
                backLeft = new MagSwerveModule(
                    kBLDriveID,
                    kBLTurningID,
                    kBLDriveReversed,
                    kBLTurningReversed,
                    MagEncoderConstants.kBLAbsoluteID,
                    MagEncoderConstants.kBLDefaultOffsetTicks,
                    MagEncoderConstants.kBLAbsoluteReversed,
                    kBLGearRatio,
                    kBLMaxSpeed);
                backRight = new MagSwerveModule(
                    kBRDriveID,
                    kBRTurningID,
                    kBRDriveReversed,
                    kBRTurningReversed,
                    MagEncoderConstants.kBRAbsoluteID,
                    MagEncoderConstants.kBRDefaultOffsetTicks,
                    MagEncoderConstants.kBRAbsoluteReversed,
                    kBRGearRatio,
                    kBRMaxSpeed);
                break;
            case CANCODER:
                frontLeft = new CANSwerveModule(
                    kFLDriveID,
                    kFLTurningID,
                    kFLDriveReversed,
                    kFLTurningReversed,
                    CANCoderConstants.kFLCANCoderID,
                    CANCoderConstants.kFLDefaultOffsetDegrees,
                    CANCoderConstants.kFLCANCoderReversed,
                    kFLGearRatio,
                    kFLMaxSpeed);
                frontRight = new CANSwerveModule(
                    kFRDriveID,
                    kFRTurningID,
                    kFRDriveReversed,
                    kFRTurningReversed,
                    CANCoderConstants.kFRCANCoderID,
                    CANCoderConstants.kFRDefaultOffsetDegrees,
                    CANCoderConstants.kFRCANCoderReversed,
                    kFRGearRatio,
                    kFRMaxSpeed);
                backLeft = new CANSwerveModule(
                    kBLDriveID,
                    kBLTurningID,
                    kBLDriveReversed,
                    kBLTurningReversed,
                    CANCoderConstants.kBLCANCoderID,
                    CANCoderConstants.kBLDefaultOffsetDegrees,
                    CANCoderConstants.kBLCANCoderReversed,
                    kBLGearRatio,
                    kBLMaxSpeed);
                backRight = new CANSwerveModule(
                    kBRDriveID,
                    kBRTurningID,
                    kBRDriveReversed,
                    kBRTurningReversed,
                    CANCoderConstants.kBRCANCoderID,
                    CANCoderConstants.kBRDefaultOffsetDegrees,
                    CANCoderConstants.kBRCANCoderReversed,
                    kBRGearRatio,
                    kBRMaxSpeed);
                break;
            default:
                throw new IllegalArgumentException("Swerve Module Type not provided");
        }

        numEncoderResets = 0;
        SmartDashboard.putNumber("Encoder resets", 0);
        resetEncoders();
        this.gyro = gyro;
        this.odometer = new SwerveDriveOdometry(
            kDriveKinematics, 
            new Rotation2d(0), 
            getModulePositions()); 
        
        field = new Field2d();
        field.setRobotPose(odometer.getPoseMeters());
    }

    /**
     * Periodically update the odometry based on the state of each module
     */
    @Override
    public void periodic() {
        reportToSmartDashboard();
        // SwerveModulePosition[] modules = getModulePositions();
        odometer.update(gyro.getRotation2d(), getModulePositions());
        field.setRobotPose(odometer.getPoseMeters());
    }
    
    //****************************** RESETTERS ******************************/


    /**
     * Resets the odometry to given pose 
     * @param pose  A Pose2D representing the pose of the robot
     */
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }

    /**
     * Reset the turning encoders on each swerve module. 
     * See {@link MagSwerveModule#resetEncoder() resetEncoder} for more details.
     */
    public void resetEncoders() {
        numEncoderResets += 1;
        SmartDashboard.putNumber("Encoder resets", numEncoderResets);
        // SmartDashboard.putNumber("Encoder resets", SmartDashboard.getNumber("Encoder resets", 0)+1);
        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        backLeft.resetEncoder();
        backRight.resetEncoder();
    }

    /**
     * Set all encoders to 0 degrees.
     */
    public void calibrateEncoders() {
        frontLeft.calibrateEncoder();
        frontRight.calibrateEncoder();
        backLeft.calibrateEncoder();
        backRight.calibrateEncoder();
    }

    /**
     * Reset all encoders to their default angle offsets.
     */
    public void resetEncodersToDefault() {
        frontLeft.resetEncoderToDefault();
        frontRight.resetEncoderToDefault();
        backLeft.resetEncoderToDefault();
        backRight.resetEncoderToDefault();
    }

    /**
     * Stops all modules. See {@link MagSwerveModule#stop()} for more info.
     */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    //****************************** GETTERS ******************************/

    public Imu getImu() {
        return this.gyro;
    }

    /**
     * Gets a pose2d representing the position of the drivetrain
     * @return A pose2d representing the position of the drivetrain
     */
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    /**
     * Get the position of each swerve module
     * @return An array of swerve module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(), 
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    //****************************** SETTERS ******************************/

    public void drive(double xSpeed, double ySpeed, double turnSpeed) {
        setModuleStates(
            SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(xSpeed, ySpeed, turnSpeed)
            )
        );
    }

    public void drive(double xSpeed, double ySpeed) {
        drive(xSpeed, ySpeed, 0);
    }

    public void driveFieldOriented(double xSpeed, double ySpeed, double turnSpeed) {
        setModuleStates(
            SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, gyro.getRotation2d())
            )
        );
    }

    public void driveFieldOriented(double xSpeed, double ySpeed) {
        driveFieldOriented(xSpeed, ySpeed, 0);
    }

    /**
     * Set the neutral modes of all modules.
     * <p>
     * true sets break mode, false sets coast mode
     * 
     * @param breaking  Whether or not the modules should be in break
     */
    public void setBreak(boolean breaking) {
        frontLeft.setBreak(breaking);
        frontRight.setBreak(breaking);
        backLeft.setBreak(breaking);
        backRight.setBreak(breaking);
    }

    /**
     * Sets module desired states
     * @param desiredStates desired states of the four modules (FL, FR, BL, BR)
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Reset the odometry to the specified position.
     * @param pose
     */
    public void setPoseMeters(Pose2d pose) {
        odometer.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }

    public void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

        tab.add("Field Position", field).withSize(6, 3);
        tab.addNumber("X Position", odometer.getPoseMeters()::getX);
        // Might be negative because our swerveDriveKinematics is flipped across the Y axis
        tab.addNumber("Y Position", odometer.getPoseMeters()::getY);
        tab.add("Reset Encoders", runOnce(this::resetEncoders));
        tab.add("Calibrate Encoders", runOnce(this::calibrateEncoders));
        tab.add("Reset Encoders to Default", runOnce(this::resetEncodersToDefault));
    }

    public void initModuleShuffleboard() {
        frontRight.initShuffleboard();
        backRight.initShuffleboard();
        frontLeft.initShuffleboard();
        backLeft.initShuffleboard();
    }

    /**
     * Report values to smartdashboard.
     */
    public void reportToSmartDashboard() {
        SmartDashboard.putNumber("Odometer X Meters", odometer.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometer Y Meters", odometer.getPoseMeters().getY());
    }

    public void reportModulesToSmartDashboard() {
        frontRight.reportToSmartDashboard();
        backRight.reportToSmartDashboard();
        frontLeft.reportToSmartDashboard();
        backLeft.reportToSmartDashboard();
    }
}
