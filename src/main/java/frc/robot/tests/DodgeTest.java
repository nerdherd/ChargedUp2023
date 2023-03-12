package frc.robot.tests;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.SwerveDriveConstants.*;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.filters.Filter;
import frc.robot.filters.NewDriverFilter;
import frc.robot.util.BadPS4;

public class DodgeTest {
    private Filter xFilter, yFilter, turningFilter;
    private Translation2d rotationCenter;

    public BadPS4 controller = new BadPS4(0);
    public Field2d field = new Field2d();

    public SwerveModulePosition[] modulePositions = 
        new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };

    public SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        SwerveDriveConstants.kDriveKinematics, new Rotation2d(), 
        modulePositions, new Pose2d());
    

    public void initialize() {
        field.setRobotPose(new Pose2d());

        this.xFilter = new NewDriverFilter(
            OIConstants.kDeadband, 
            kMinimumMotorOutput,
            4, 
            kDriveAlpha, 
            kTeleMaxAcceleration, 
            kTeleMaxDeceleration);
        this.yFilter = new NewDriverFilter(
            OIConstants.kDeadband, 
            kMinimumMotorOutput,
            4, 
            kDriveAlpha, 
            kTeleMaxAcceleration, 
            kTeleMaxDeceleration);
        this.turningFilter = new NewDriverFilter(
            OIConstants.kDeadband, 
            kMinimumMotorOutput,
            kTeleDriveMaxAngularSpeedRadiansPerSecond, 
            kDriveAlpha, 
            kTeleMaxAcceleration, 
            kTeleMaxDeceleration);
    }

    public void execute() {
        // get speeds
        double turningSpeed = controller.getRightX();
        double xSpeed = controller.getLeftX();
        double ySpeed = -controller.getLeftY();

        double filteredTurningSpeed = turningFilter.calculate(turningSpeed);
        double filteredXSpeed = xFilter.calculate(xSpeed);
        double filteredYSpeed = yFilter.calculate(ySpeed);
        
        
        ChassisSpeeds chassisSpeeds;
        // Check if in field oriented mode
            SmartDashboard.putString("Mode", "Field Oriented");
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                filteredXSpeed, filteredYSpeed, filteredTurningSpeed, 
                poseEstimator.getEstimatedPosition().getRotation());
                
        SmartDashboard.putNumber("Swerve Drive X Speed", filteredXSpeed);
        SmartDashboard.putNumber("Swerve Drive Y Speed", filteredYSpeed);
        SmartDashboard.putNumber("Swerve Drive X Chassis", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Swerve Drive Y Chassis", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Turning speed", filteredTurningSpeed);

        SwerveModuleState[] moduleStates;

        if (controller.getCircleButton()) {
            if (rotationCenter == null) {
                rotationCenter = new Translation2d(kRotationOffset, 
                    // Rotation 2d is measured counterclockwise from the right vector
                    // Y speed = left/right = x component
                    // X speed = forward/back = y component
                    new Rotation2d(ySpeed, xSpeed)
                        .rotateBy(
                            // imu is measured clockwise from forward vector
                            poseEstimator.getEstimatedPosition().getRotation())
                        );
                // Might need to swap x and y on rotation center depending on how it gets interpreted
                rotationCenter = new Translation2d(rotationCenter.getY(), rotationCenter.getX());
                SmartDashboard.putString("Rotation Center", rotationCenter.toString());
            }
            moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds, rotationCenter);
        } else {
            rotationCenter = null;
            moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        }

        for (int i = 0; i<4; i++) {
            modulePositions[i].distanceMeters = modulePositions[i].distanceMeters + 0.02*moduleStates[i].speedMetersPerSecond;
            modulePositions[i].angle = moduleStates[i].angle;
        }


        poseEstimator.update(poseEstimator.getEstimatedPosition().getRotation(), modulePositions);
        field.setRobotPose(poseEstimator.getEstimatedPosition());
        SmartDashboard.putData(field);
    }
}
