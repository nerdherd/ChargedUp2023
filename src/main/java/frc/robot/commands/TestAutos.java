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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

import static frc.robot.Constants.SwerveAutoConstants.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class TestAutos {
    public static CommandBase testAuto1(SwerveDrivetrain swerveDrive) {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kChargeSpeedMetersPerSecond / 2, 
            kChargeAccelerationMetersPerSecondSquared / 2);

        Trajectory goForward = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(3.5, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(4, 0.5, Rotation2d.fromDegrees(180)),
                new Pose2d(2, 0, Rotation2d.fromDegrees(180))
            ),
            trajectoryConfig);

        // Trajectory comeBack = TrajectoryGenerator.generateTrajectory(
        //     List.of(
        //         new Pose2d(4, 0, Rotation2d.fromDegrees(0)),
        //         new Pose2d(2, 0.01, Rotation2d.fromDegrees(0))
        //     ),
        //     trajectoryConfig);
        
        PIDController xController = new PIDController(kPXController, kIXController, kDXController);
        PIDController yController = new PIDController(kPYController, kIYController, kDYController);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand goForwardCommand = new SwerveControllerCommand(
            goForward, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        // SwerveControllerCommand comeBackCommand = new SwerveControllerCommand(
        //     comeBack, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
        //     xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return sequence(
            runOnce(() -> swerveDrive.resetOdometry(goForward.getInitialPose())),
            runOnce(() -> swerveDrive.setVelocityControl(true)),
            runOnce(() -> SmartDashboard.putString("Stage", "Going forward")),
            goForwardCommand,
            // runOnce(() -> {
            //     xController.reset();
            //     yController.reset();
            // }),
            // runOnce(() -> SmartDashboard.putString("Stage", "Went forward")),
            // runOnce(() -> swerveDrive.stopModules()),
            // waitSeconds(1),
            // runOnce(() -> SmartDashboard.putString("Stage", "Coming back")),
            // comeBackCommand,
            runOnce(() -> SmartDashboard.putString("Stage", "Finished")),
            runOnce(() -> swerveDrive.stopModules())
        );
    }
}
