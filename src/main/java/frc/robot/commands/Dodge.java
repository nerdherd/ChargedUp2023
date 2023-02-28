package frc.robot.commands;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.SwerveDriveConstants;
import static frc.robot.Constants.SwerveAutoConstants.*;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class Dodge extends SwerveControllerCommand {
    private static boolean facingForward(double angle) {
        if (angle >= 270 || angle <= 90) {
            return true;
        } else {
            return false;
        }
    }

    private static ProfiledPIDController getAngleController() {
        ProfiledPIDController controller = new ProfiledPIDController(
            kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);
        
        controller.enableContinuousInput(-Math.PI, Math.PI);

        return controller;
    }

    private static Trajectory getDodgeTrajectory(SwerveDrivetrain swerveDrive, double xSpeed, double ySpeed, boolean isLeft, double distance) {
        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxAutoSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);

        double angle;

        if (xSpeed < 0.01 && ySpeed < 0.01) {
            // Assume that the robot is facing forward
            angle = 0;
        } else {
            // The direction that the joystick is pointing towards (in robot oriented)
            Translation2d rotationCenter = new Translation2d(
                SwerveDriveConstants.kRotationOffset, 
                // Rotation 2d is measured counterclockwise from the right vector
                // Y speed = left/right = x component
                // X speed = forward/back = y component
                new Rotation2d(ySpeed, xSpeed)
                    .rotateBy(Rotation2d.fromDegrees(
                        // imu is measured clockwise from forward vector
                        swerveDrive.getImu().getHeading()))
                    );
            
            angle = -Math.atan2(rotationCenter.getX(), rotationCenter.getY());
        }

        angle = 0;
                
        // Get angle of robot relative to joystick direction
        double endAngle = angle;

        
        boolean facingForward = facingForward(swerveDrive.getImu().getHeading());
        
        facingForward = true;

        double yTranslation = distance;

        // Up-Left or Down-Right
        if (isLeft && facingForward || !isLeft && !facingForward) {
            endAngle += (Math.PI / 2);
            yTranslation *= -1;
        } else {
        // Up-right or Down-Left
            endAngle -= (Math.PI / 2);
        }

        // TODO: Change to robot oriented (does not work properly without)
        // Either to the top left or bottom right
        Trajectory dodgeTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(angle)), 
            List.of(
                new Translation2d(0, yTranslation)
            ),
            new Pose2d(distance, yTranslation, new Rotation2d(endAngle)), 
            trajectoryConfig);
        
        return dodgeTrajectory;
    }

    public Dodge(SwerveDrivetrain swerveDrive, double xSpeed, double ySpeed, boolean isLeft, double distance) {    
        super(
            getDodgeTrajectory(swerveDrive, xSpeed, ySpeed, isLeft, distance), 
            swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            new PIDController(kPXController, kIXController, kDXController),
            new PIDController(kPYController, kIYController, kDYController),
            getAngleController(),
            swerveDrive::setModuleStates, 
            swerveDrive
        );
    }

    public Dodge(SwerveDrivetrain swerveDrive, double xSpeed, double ySpeed, boolean isLeft) {
        this(swerveDrive, xSpeed, ySpeed, isLeft, SwerveDriveConstants.kDodgeDistance);
    }
}
