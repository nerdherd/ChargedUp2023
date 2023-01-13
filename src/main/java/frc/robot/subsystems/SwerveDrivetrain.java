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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;

public class SwerveDrivetrain extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            SwerveDriveConstants.kFrontLeftDriveMotorPort,
            SwerveDriveConstants.kFrontLeftTurningMotorPort,
            SwerveDriveConstants.kFrontLeftDriveMotorReversed,
            SwerveDriveConstants.kFrontLeftTurningMotorReversed,
            SwerveDriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            SwerveDriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            SwerveDriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            SwerveDriveConstants.kFrontRightDriveMotorPort,
            SwerveDriveConstants.kFrontRightTurningMotorPort,
            SwerveDriveConstants.kFrontRightDriveMotorReversed,
            SwerveDriveConstants.kFrontRightTurningMotorReversed,
            SwerveDriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            SwerveDriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            SwerveDriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            SwerveDriveConstants.kBackLeftDriveMotorPort,
            SwerveDriveConstants.kBackLeftTurningMotorPort,
            SwerveDriveConstants.kBackLeftDriveMotorReversed,
            SwerveDriveConstants.kBackLeftTurningMotorReversed,
            SwerveDriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            SwerveDriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            SwerveDriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            SwerveDriveConstants.kBackRightDriveMotorPort,
            SwerveDriveConstants.kBackRightTurningMotorPort,
            SwerveDriveConstants.kBackRightDriveMotorReversed,
            SwerveDriveConstants.kBackRightTurningMotorReversed,
            SwerveDriveConstants.kBackRightDriveAbsoluteEncoderPort,
            SwerveDriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            SwerveDriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro;
    private final SwerveDriveOdometry odometer;

    /**
     * Construct a new {@link SwerveDriveTrain}
     */
    public SwerveDrivetrain() {
        SmartDashboard.putNumber("Gyro resets", 0);

        this.gyro = new AHRS(SPI.Port.kMXP);
        this.odometer = new SwerveDriveOdometry(
            SwerveDriveConstants.kDriveKinematics, 
            new Rotation2d(0), 
            getModulePositions());
        new Thread(() -> {
            try {
                resetEncoders();
                Thread.sleep(1000);
                zeroHeading();
                SmartDashboard.putNumber("Gyro resets", 1);
                SmartDashboard.putBoolean("Startup failed", false);
            } catch (InterruptedException e) {
                SmartDashboard.putBoolean("Startup failed", true);
            }
        }).run();
    }

    /**
     * Periodically update the odometry based on the state of each module
     */
    @Override
    public void periodic() {
        reportToSmartDashboard();
        odometer.update(getRotation2d(), getModulePositions());
    }
    
    //****************************** RESETTERS ******************************/

    /**
     * Set the current gyro direction to north
     */
    public void zeroHeading() {
        gyro.reset();
        SmartDashboard.putNumber("Gyro resets", SmartDashboard.getNumber("Gyro resets", 0)+1);
    }

    /**
     * Resets the odometry to given pose 
     * @param pose  A Pose2D representing the pose of the robot
     */
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    /**
     * Reset the turning encoders on each swerve module. 
     * See {@link SwerveModule#resetEncoder() resetEncoder} for more details.
     */
    public void resetEncoders() {
        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        backLeft.resetEncoder();
        backRight.resetEncoder();
    }

    /**
     * Stops all modules. See {@link SwerveModule#stop()} for more info.
     */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    //****************************** GETTERS ******************************/

    /**
     * Gets angle robot is facing
     * @return Angle of the robot (degrees)
     */
    public double getHeading() {
        double heading = Math.IEEEremainder(gyro.getAngle(), 360);
        SmartDashboard.putNumber("Heading degrees", heading);
        return heading;
    }

    /**
     * Gets a rotation2d representing rotation of the drivetrain
     * @return A rotation2d representing rotation of the drivetrain
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Rotation3d getRotation3d() {
        return new Rotation3d(
            gyro.getRoll() * Math.PI / 180, 
            gyro.getPitch()* Math.PI / 180, 
            gyro.getYaw() * Math.PI / 180) ;
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
    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(), 
            frontRight.getPosition(), 
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    //****************************** SETTERS ******************************/

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
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveDriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Report values to smartdashboard.
     */
    public void reportToSmartDashboard() {
        SmartDashboard.putNumber("xpos", getPose().getX());
        SmartDashboard.putNumber("ypos", getPose().getY());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }
}
