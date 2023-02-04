package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;

import static frc.robot.Constants.SwerveDriveConstants.*;

public class SwerveDrivetrain extends SubsystemBase implements Reportable {
    private final SwerveModule frontLeftOld = new SwerveModule(
            kFLDriveID,
            kFLTurningID,
            kFLDriveReversed,
            kFLTurningReversed,
            kFLAbsoluteID,
            kFLAbsoluteOffsetTicks,
            kFLAbsoluteReversed);

    private final SwerveModule frontRightOld = new SwerveModule(
            kFRDriveID,
            kFRTurningID,
            kFRDriveReversed,
            kFRTurningReversed,
            kFRAbsoluteID,
            kFRAbsoluteOffsetTicks,
            kFRAbsoluteReversed);

    private final SwerveModule backLeftOld = new SwerveModule(
            kBLDriveID,
            kBLTurningID,
            kBLDriveReversed,
            kBLTurningReversed,
            kBLAbsoluteID,
            kBLAbsouteOffsetTicks,
            kBLAbsoluteReversed);

    private final SwerveModule backRightOld = new SwerveModule(
            kBRDriveID,
            kBRTurningID,
            kBRDriveReversed,
            kBRTurningReversed,
            kBRAbsoluteID,
            kBRAbsoluteOffsetTicks,
            kBRAbsoluteReversed);

    private final Imu gyro;
    private final SwerveDriveOdometry odometer;

    private int numEncoderResets = 0;

    /**
     * Construct a new {@link SwerveDrivetrain}
     */
    public SwerveDrivetrain(Imu gyro) {
        numEncoderResets = 0;
        SmartDashboard.putNumber("Encoder resets", 0);

        this.gyro = gyro;
        this.odometer = new SwerveDriveOdometry(
            kDriveKinematics, 
            new Rotation2d(0), 
            getModulePositions());   
    }

    /**
     * Periodically update the odometry based on the state of each module
     */
    @Override
    public void periodic() {
        reportToSmartDashboard();
        // SwerveModulePosition[] modules = getModulePositions();
        odometer.update(gyro.getRotation2d(), getModulePositions());
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
     * See {@link SwerveModule#resetEncoder() resetEncoder} for more details.
     */
    public void resetEncoders() {
        numEncoderResets += 1;
        SmartDashboard.putNumber("Encoder resets", numEncoderResets);
        // SmartDashboard.putNumber("Encoder resets", SmartDashboard.getNumber("Encoder resets", 0)+1);
        frontLeftOld.resetEncoder();
        frontRightOld.resetEncoder();
        backLeftOld.resetEncoder();
        backRightOld.resetEncoder();
    }

    /**
     * Stops all modules. See {@link SwerveModule#stop()} for more info.
     */
    public void stopModules() {
        frontLeftOld.stop();
        frontRightOld.stop();
        backLeftOld.stop();
        backRightOld.stop();
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
            frontLeftOld.getPosition(),
            frontRightOld.getPosition(), 
            backLeftOld.getPosition(),
            backRightOld.getPosition()
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
        frontLeftOld.setBreak(breaking);
        frontRightOld.setBreak(breaking);
        backLeftOld.setBreak(breaking);
        backRightOld.setBreak(breaking);
    }

    /**
     * Sets module desired states
     * @param desiredStates desired states of the four modules (FL, FR, BL, BR)
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kPhysicalMaxSpeedMetersPerSecond);
        frontLeftOld.setDesiredState(desiredStates[0]);
        frontRightOld.setDesiredState(desiredStates[1]);
        backLeftOld.setDesiredState(desiredStates[2]);
        backRightOld.setDesiredState(desiredStates[3]);
    }

    /**
     * Report values to smartdashboard.
     */
    public void reportToSmartDashboard() {
        SmartDashboard.putNumber("Odometer X Meters", odometer.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometer Y Meters", odometer.getPoseMeters().getY());
    }

    public void reportModulesToSmartDashboard() {
        frontRightOld.reportToSmartDashboard();
        backRightOld.reportToSmartDashboard();
        frontLeftOld.reportToSmartDashboard();
        backLeftOld.reportToSmartDashboard();
    }
}
