package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrivetrain;

import static frc.robot.Constants.SwerveAutoConstants.*;

import java.util.List;

public class SwerveAutos {
    public static CommandBase testAuto(SwerveDrivetrain swerveDrive) {
        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
        // Create Actual Trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
            new Translation2d(1, 0),
            new Translation2d(1, 1),
            new Translation2d(0, 1)), 
            new Pose2d(0, 0, new Rotation2d(0)), 
            trajectoryConfig);
    
        //Create PID Controllers
        PIDController xController = new PIDController(kPXController, 0, 0);
        PIDController yController = new PIDController(kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, 0, 0, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand autoCommand = new SwerveControllerCommand(
            trajectory, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return Commands.sequence(
            Commands.runOnce(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
            autoCommand, 
            Commands.runOnce(() -> swerveDrive.stopModules()));
    } 
}
