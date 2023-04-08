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

    public static CommandBase taxiSLOW(SwerveDrivetrain swerveDrive) {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            2, 
            3);

        Trajectory goForward = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(5.25, 0.5, Rotation2d.fromDegrees(0))
            ),
            trajectoryConfig);
        
        Trajectory comeBack = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(5.25, 0.5, Rotation2d.fromDegrees(0)),
                new Pose2d(2, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(0, 0.75, Rotation2d.fromDegrees(180))
            ),
            trajectoryConfig);
        
        Trajectory goBack = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0.75, Rotation2d.fromDegrees(180)),
                new Pose2d(2, 0, Rotation2d.fromDegrees(180)),
                new Pose2d(5.25, 1.75, Rotation2d.fromDegrees(180))
            ),
            trajectoryConfig);

        PIDController xController = new PIDController(kPXController, kIXController, kDXController);
        PIDController yController = new PIDController(kPYController, kIYController, kDYController);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand goForwardCommand = new SwerveControllerCommand(
            goForward, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);

        SwerveControllerCommand comeBackCommand = new SwerveControllerCommand(
            comeBack, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);

        SwerveControllerCommand goBackCommand = new SwerveControllerCommand(
            goBack, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return sequence(
            runOnce(() -> swerveDrive.resetOdometry(goForward.getInitialPose())),
            runOnce(() -> swerveDrive.setVelocityControl(true)),
            runOnce(() -> SmartDashboard.putString("Stage", "Taxiing")),
            goForwardCommand,
            runOnce(() -> SmartDashboard.putString("Stage", "Coming Back")),
            runOnce(() -> swerveDrive.stopModules()),
            waitSeconds(2),
            // new TurnToAngle(180, swerveDrive),
            comeBackCommand,
            runOnce(() -> swerveDrive.stopModules()),
            waitSeconds(1),
            goBackCommand,
            runOnce(() -> SmartDashboard.putString("Stage", "Done")),
            runOnce(() -> swerveDrive.stopModules())
        );
    }

    public static CommandBase taxiChargeBackwards(SwerveDrivetrain swerveDrive) {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            1, 
            1);

        Trajectory goForward = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
                // new Pose2d(3.5, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(4, 0.01, Rotation2d.fromDegrees(180))
                // new Pose2d(2.5, 0.01, Rotation2d.fromDegrees(180))
            ),
            trajectoryConfig);
        Trajectory goBackward = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(4, 0, Rotation2d.fromDegrees(180)),
                new Pose2d(1.75, 0.01, Rotation2d.fromDegrees(180))
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
        
        SwerveControllerCommand comeBackCommand = new SwerveControllerCommand(
            goBackward, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return sequence(
            parallel(
                runOnce(() -> swerveDrive.resetOdometry(goForward.getInitialPose())),
                runOnce(() -> swerveDrive.setVelocityControl(true)),
                runOnce(() -> SmartDashboard.putString("Stage", "Going forward")),
                runOnce(() -> swerveDrive.getImu().zeroRoll()),
                runOnce(() -> swerveDrive.getImu().zeroPitch())
            ),
            goForwardCommand,
            runOnce(() -> swerveDrive.stopModules()),
            // waitSeconds(1),
            // runOnce(() -> {
            //     xController.reset();
            //     yController.reset();
            // }),
            // runOnce(() -> SmartDashboard.putString("Stage", "Went forward")),
            // runOnce(() -> swerveDrive.stopModules()),
            // waitSeconds(1),
            // runOnce(() -> SmartDashboard.putString("Stage", "Coming back")),
            // new TurnToAngle(180, swerveDrive),
            comeBackCommand,
            runOnce(() -> SmartDashboard.putString("Stage", "Balancing")),
            race(
                waitSeconds(2),
                new TheGreatBalancingAct(swerveDrive, 2, 0.0, 0.1, 0.0, 0.0, 0.0)
            ),
            runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
            runOnce(() -> swerveDrive.stopModules())
        );
    }

    public static CommandBase chargeBackwardsSLOW(SwerveDrivetrain swerveDrive) {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            1, 
            1);

        Trajectory goForward = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
                new Pose2d(2, 0.01, Rotation2d.fromDegrees(180))
            ),
            trajectoryConfig);
        
        PIDController xController = new PIDController(kPXController, kIXController, kDXController);
        PIDController yController = new PIDController(kPYController, kIYController, kDYController);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand goForwardCommand = new SwerveControllerCommand(
            goForward, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return sequence(
            parallel(
                runOnce(() -> swerveDrive.resetOdometry(goForward.getInitialPose())),
                runOnce(() -> swerveDrive.setVelocityControl(true)),
                runOnce(() -> SmartDashboard.putString("Stage", "Going forward")),
                runOnce(() -> swerveDrive.getImu().zeroRoll()),
                runOnce(() -> swerveDrive.getImu().zeroPitch())
            ),
            goForwardCommand,
            runOnce(() -> SmartDashboard.putString("Stage", "Balancing")),
            race(
                waitSeconds(2),
                new TheGreatBalancingAct(swerveDrive, 2, 0.0, 0.1, 0.0, 0.0, 0.0)
            ),
            runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
            runOnce(() -> swerveDrive.stopModules())
        );
    }

    public static CommandBase taxiChargeForwards(SwerveDrivetrain swerveDrive) {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            1, 
            1);

        Trajectory goForward = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                // new Pose2d(3.5, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(4.5, 0.01, Rotation2d.fromDegrees(0))
                // new Pose2d(2.5, 0.01, Rotation2d.fromDegrees(180))
            ),
            trajectoryConfig);
        Trajectory goBackward = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(4.5, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(2, 0.01, Rotation2d.fromDegrees(0))
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
        
        SwerveControllerCommand comeBackCommand = new SwerveControllerCommand(
            goBackward, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return sequence(
            parallel(
                runOnce(() -> swerveDrive.resetOdometry(goForward.getInitialPose())),
                runOnce(() -> swerveDrive.setVelocityControl(true)),
                runOnce(() -> SmartDashboard.putString("Stage", "Going forward")),
                runOnce(() -> swerveDrive.getImu().zeroRoll()),
                runOnce(() -> swerveDrive.getImu().zeroPitch())
            ),
            goForwardCommand,
            runOnce(() -> swerveDrive.stopModules()),
            waitSeconds(1),
            // runOnce(() -> {
            //     xController.reset();
            //     yController.reset();
            // }),
            // runOnce(() -> SmartDashboard.putString("Stage", "Went forward")),
            // runOnce(() -> swerveDrive.stopModules()),
            // waitSeconds(1),
            // runOnce(() -> SmartDashboard.putString("Stage", "Coming back")),
            // new TurnToAngle(180, swerveDrive),
            comeBackCommand,
            runOnce(() -> SmartDashboard.putString("Stage", "Balancing")),
            race(
                waitSeconds(2),
                new TheGreatBalancingAct(swerveDrive, 2, 0.0, 0.1, 0.0, 0.0, 0.0)
            ),
            runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
            runOnce(() -> swerveDrive.stopModules())
        );
    }

}
