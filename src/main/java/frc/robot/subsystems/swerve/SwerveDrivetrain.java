package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveDriveConstants.CANCoderConstants;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.vision.primalWallnut.PrimalSunflower;
import frc.robot.subsystems.Reportable;

import static frc.robot.Constants.SwerveDriveConstants.*;

public class SwerveDrivetrain extends SubsystemBase implements Reportable {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final Gyro gyro;
    // private final SwerveDriveOdometry odometer;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final PrimalSunflower sunflower;
    private DRIVE_MODE driveMode = DRIVE_MODE.FIELD_ORIENTED;

    private Field2d field;

    private int numEncoderResets = 0;

    public enum SwerveModuleType {
        MAG_ENCODER,
        CANCODER
    }

    public enum DRIVE_MODE {
        FIELD_ORIENTED,
        ROBOT_ORIENTED,
        AUTONOMOUS
    }

    /**
     * Construct a new {@link SwerveDrivetrain}
     */
    public SwerveDrivetrain(Gyro gyro, SwerveModuleType moduleType, PrimalSunflower sunflower) throws IllegalArgumentException {
        switch (moduleType) {
            case CANCODER:
                frontLeft = new CANSwerveModule(
                    kFLDriveID,
                    kFLTurningID,
                    kFLDriveReversed,
                    kFLTurningReversed,
                    CANCoderConstants.kFLCANCoderID,
                    CANCoderConstants.kFLOffsetDeg.get(),
                    CANCoderConstants.kFLCANCoderReversed);
                frontRight = new CANSwerveModule(
                    kFRDriveID,
                    kFRTurningID,
                    kFRDriveReversed,
                    kFRTurningReversed,
                    CANCoderConstants.kFRCANCoderID,
                    CANCoderConstants.kFROffsetDeg.get(),
                    CANCoderConstants.kFRCANCoderReversed);
                backLeft = new CANSwerveModule(
                    kBLDriveID,
                    kBLTurningID,
                    kBLDriveReversed,
                    kBLTurningReversed,
                    CANCoderConstants.kBLCANCoderID,
                    CANCoderConstants.kBLOffsetDeg.get(),
                    CANCoderConstants.kBLCANCoderReversed);
                backRight = new CANSwerveModule(
                    kBRDriveID,
                    kBRTurningID,
                    kBRDriveReversed,
                    kBRTurningReversed,
                    CANCoderConstants.kBRCANCoderID,
                    CANCoderConstants.kBROffsetDeg.get(),
                    CANCoderConstants.kBRCANCoderReversed);
                break;
            default:
                throw new IllegalArgumentException("Invalid Swerve Module Type provided.");
        }

        numEncoderResets = 0;
        resetEncoders();
        this.gyro = gyro;
        this.poseEstimator = new SwerveDrivePoseEstimator(kDriveKinematics, gyro.getRotation2d(), getModulePositions(), new Pose2d());
        this.poseEstimator.setVisionMeasurementStdDevs(kBaseVisionPoseSTD);
        this.sunflower = sunflower;
        // this.odometer = new SwerveDriveOdometry(
        //     kDriveKinematics, 
        //     new Rotation2d(0), 
        //     getModulePositions()); 
        
        field = new Field2d();
        field.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    /**
     * Have modules move towards states and update odometry
     */
    @Override
    public void periodic() {
        runModules();
        // odometer.update(gyro.getRotation2d(), getModulePositions());
        poseEstimator.update(gyro.getRotation2d(), getModulePositions());

        // === Removed vision to prevent CPU lag. To add back, uncomment theh following 5 lines. ===

        // Pose3d sunflowerPose3d = sunflower.getPose3d();
        // if (sunflowerPose3d != null) {
        //     poseEstimator.addVisionMeasurement(sunflowerPose3d.toPose2d(), Timer.getFPGATimestamp());
        // }
        // field.setRobotPose(poseEstimator.getEstimatedPosition());
    }
    
    //****************************** RESETTERS ******************************/


    /**
     * Resets the odometry to given pose 
     * @param pose  A Pose2D representing the pose of the robot
     */
    public void resetOdometry(Pose2d pose) {
        // odometer.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
        poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }

    /**
     * Reset the turning encoders on each swerve module. 
     * See {@link MagSwerveModule#resetEncoder() resetEncoder} for more details.
     */
    public void resetEncoders() {
        numEncoderResets += 1;
        // SmartDashboard.putNumber("Encoder resets", numEncoderResets);
        CANCoderConstants.kFLOffsetDeg.loadPreferences();
        CANCoderConstants.kFROffsetDeg.loadPreferences();
        CANCoderConstants.kBLOffsetDeg.loadPreferences();
        CANCoderConstants.kBROffsetDeg.loadPreferences();
        frontLeft.setTurnOffset(CANCoderConstants.kFLOffsetDeg.get());
        frontRight.setTurnOffset(CANCoderConstants.kFROffsetDeg.get());
        backLeft.setTurnOffset(CANCoderConstants.kBLOffsetDeg.get());
        backRight.setTurnOffset(CANCoderConstants.kBROffsetDeg.get());
        
        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        backLeft.resetEncoder();
        backRight.resetEncoder();
    }

    public void zeroModules() {
        CANCoderConstants.kFLOffsetDeg.set(frontLeft.getTurnOffset() + frontLeft.getTurningPosition());
        CANCoderConstants.kFROffsetDeg.set(frontRight.getTurnOffset() + frontRight.getTurningPosition());
        CANCoderConstants.kBLOffsetDeg.set(backLeft.getTurnOffset() + backLeft.getTurningPosition());
        CANCoderConstants.kBROffsetDeg.set(backRight.getTurnOffset() + backRight.getTurningPosition());
        
        resetEncoders();
    }

    /**
     * Stops all modules. See {@link CANSwerveModule#stop()} for more info.
     */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Have modules move to their desired states. See {@link CANSwerveModule#run()} for more info.
     */
    public void runModules() {
        frontLeft.run();
        frontRight.run();
        backLeft.run();
        backRight.run();
    }

    //****************************** GETTERS ******************************/

    public Gyro getImu() {
        return this.gyro;
    }

    /**
     * Gets a pose2d representing the position of the drivetrain
     * @return A pose2d representing the position of the drivetrain
     */
    public Pose2d getPose() {
        // return odometer.getPoseMeters();
        return poseEstimator.getEstimatedPosition();
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

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] targetStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(targetStates);
    }

    //****************************** SETTERS ******************************/

    /**
     * Set the drive mode (only for telemetry purposes)
     * @param driveMode
     */
    public void setDriveMode(DRIVE_MODE driveMode) {
        this.driveMode = driveMode;
    }

    public void setVelocityControl(boolean withVelocityControl) {
        frontLeft.toggleVelocityControl(withVelocityControl);
        frontRight.toggleVelocityControl(withVelocityControl);
        backLeft.toggleVelocityControl(withVelocityControl);
        backRight.toggleVelocityControl(withVelocityControl);
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
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void towModules() {
        frontLeft.setDesiredState(towModuleStates[0], false);
        frontRight.setDesiredState(towModuleStates[1], false);
        backLeft.setDesiredState(towModuleStates[2], false);
        backRight.setDesiredState(towModuleStates[3], false);
    }

    /**
     * Reset the odometry to the specified position.
     * @param pose
     */
    public void setPoseMeters(Pose2d pose) {
        // odometer.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
        poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }

    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF)  {
            return;
        }
        ShuffleboardTab tab;
        if (level == LOG_LEVEL.MINIMAL) {
            tab = Shuffleboard.getTab("Main");
        } else {
            tab = Shuffleboard.getTab("Swerve");
        }

        switch (level) {
            case OFF:
                break;
            case ALL:
                tab.add("Field Position", field).withSize(6, 3);
                tab.add("Zero Modules", Commands.runOnce(this::zeroModules));
                // Might be negative because our swerveDriveKinematics is flipped across the Y axis
            case MEDIUM:
                tab.addNumber("Encoder Resets", () -> this.numEncoderResets);
            case MINIMAL:
                // tab.addNumber("X Position (m)", () -> odometer.getPoseMeters().getX());
                // tab.addNumber("Y Position (m)", () -> odometer.getPoseMeters().getY());
                tab.addNumber("X Position (m)", () -> poseEstimator.getEstimatedPosition().getX());
                tab.addNumber("Y Position (m)", () -> poseEstimator.getEstimatedPosition().getY());
                tab.addString("Drive Mode", () -> this.driveMode.toString());
                break;
        }
    }

    public void initModuleShuffleboard(LOG_LEVEL level) {
        frontRight.initShuffleboard(level);
        frontLeft.initShuffleboard(level);
        backLeft.initShuffleboard(level);
        backRight.initShuffleboard(level);
    }

    /**
     * Report values to smartdashboard.
     */
    public void reportToSmartDashboard(LOG_LEVEL level) {
        switch (level) {
            case OFF:
                break;
            case ALL:
            case MEDIUM:
                SmartDashboard.putNumber("Encoder Resets", numEncoderResets);
                SmartDashboard.putData("Zero Modules", Commands.runOnce(this::zeroModules));
            case MINIMAL:
                // SmartDashboard.putNumber("Odometer X Meters", odometer.getPoseMeters().getX());
                // SmartDashboard.putNumber("Odometer Y Meters", odometer.getPoseMeters().getY());
                SmartDashboard.putNumber("Odometer X Meters", poseEstimator.getEstimatedPosition().getX());
                SmartDashboard.putNumber("Odometer Y Meters", poseEstimator.getEstimatedPosition().getY());
                SmartDashboard.putString("Drive Mode", this.driveMode.toString());
                break;
        }
    }

    public void reportModulesToSmartDashboard(LOG_LEVEL level) {
        frontRight.reportToSmartDashboard(level);
        frontLeft.reportToSmartDashboard(level);
        backLeft.reportToSmartDashboard(level);
        backRight.reportToSmartDashboard(level);
    }
}
