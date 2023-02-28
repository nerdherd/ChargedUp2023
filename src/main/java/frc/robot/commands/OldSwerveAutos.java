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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

import static frc.robot.Constants.SwerveAutoConstants.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class OldSwerveAutos {
/**
     * Start with the front left swerve module aligned with the charging station's edge
     * in the x axis and around 8 inches to the right in the y axis
     * @param swerveDrive
     * @return Command sequence to turn to grids, move back to pick up a piece, go to the grid 
     * and then onto the charging station with balancing code
     */
    public static CommandBase hardCarryAuto(SwerveDrivetrain swerveDrive) {
        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxAutoSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
        // Create Actual Trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(180)), 
            List.of(
            new Translation2d(Units.inchesToMeters(249.875) , 0)
            ),
            new Pose2d(Units.inchesToMeters(249.875), Units.inchesToMeters(16), new Rotation2d(0)), 
            trajectoryConfig);
        
        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(249.875), Units.inchesToMeters(16), new Rotation2d(0)), 
            List.of(
            new Translation2d(0, Units.inchesToMeters(16))), 
            new Pose2d(0, Units.inchesToMeters(27.875), Rotation2d.fromDegrees(180)), 
            trajectoryConfig);
                
        Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(27.875), new Rotation2d(180)), 
            List.of(
            new Translation2d(0, Units.inchesToMeters(47.625))), 
            new Pose2d(Units.inchesToMeters(80), Units.inchesToMeters(47.625), Rotation2d.fromDegrees(0)), 
            trajectoryConfig);

        //Create PID Controllers
        PIDController xController = new PIDController(kPXController, kIXController, kDXController);
        PIDController yController = new PIDController(kPYController, kIYController, kDYController);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand autoCommand = new SwerveControllerCommand(
            trajectory, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        SwerveControllerCommand autoCommand2 = new SwerveControllerCommand(
            trajectory2, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
            
        SwerveControllerCommand autoCommand3 = new SwerveControllerCommand(
            trajectory3, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);

        return sequence(
            runOnce(() -> SmartDashboard.putBoolean("called 2", true)),
            waitSeconds(2),
            runOnce(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
            new TurnToAngle(180, swerveDrive),
            waitSeconds(3),
            autoCommand,
            runOnce(() -> swerveDrive.stopModules()),
            new TurnToAngle(0, swerveDrive),
            waitSeconds(1),
            autoCommand2, 
            new TurnToAngle(180, swerveDrive),
            waitSeconds(3),
            autoCommand3,
            new TheGreatBalancingAct(swerveDrive),
            runOnce(() -> swerveDrive.stopModules()),
            new TowSwerve(swerveDrive));
    }

    public static CommandBase vendingMachine(SwerveDrivetrain swerveDrive) {
        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxAutoSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
        // Create Actual Trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(180)), 
            List.of(
            new Translation2d(Units.inchesToMeters(249.875) , 0)
            ),
            new Pose2d(Units.inchesToMeters(249.875), Units.inchesToMeters(-16), new Rotation2d(0)), 
            trajectoryConfig);
        
        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(249.875), Units.inchesToMeters(-16), new Rotation2d(0)), 
            List.of(
            new Translation2d(0, Units.inchesToMeters(-16))), 
            new Pose2d(0, Units.inchesToMeters(-27.875), Rotation2d.fromDegrees(180)), 
            trajectoryConfig);
                
        Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(-27.875), new Rotation2d(180)), 
            List.of(
            new Translation2d(0, Units.inchesToMeters(-47.625))), 
            new Pose2d(Units.inchesToMeters(80), Units.inchesToMeters(-47.625), Rotation2d.fromDegrees(0)), 
            trajectoryConfig);

        //Create PID Controllers
        PIDController xController = new PIDController(kPXController, kIXController, kDXController);
        PIDController yController = new PIDController(kPYController, kIYController, kDYController);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand autoCommand = new SwerveControllerCommand(
            trajectory, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        SwerveControllerCommand autoCommand2 = new SwerveControllerCommand(
            trajectory2, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
            
        SwerveControllerCommand autoCommand3 = new SwerveControllerCommand(
            trajectory3, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);

        return sequence(
            waitSeconds(4),
            runOnce(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
            new TurnToAngle(180, swerveDrive),
            waitSeconds(3),
            autoCommand,
            runOnce(() -> swerveDrive.stopModules()),
            new TurnToAngle(0, swerveDrive),
            waitSeconds(1),
            autoCommand2, 
            new TurnToAngle(180, swerveDrive),
            waitSeconds(3),
            autoCommand3,
            new TheGreatBalancingAct(swerveDrive),
            runOnce(() -> swerveDrive.stopModules()),
            new TowSwerve(swerveDrive));
    } 

    // public static CommandBase dietCoke(SwerveDrivetrain swerveDrive) {
    //     // Create trajectory settings
    //     TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    //         kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
    //     // Create Actual Trajectory
    //     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //         new Pose2d(0, 0, new Rotation2d(0)), 
    //         List.of(
    //         new Translation2d(Units.inchesToMeters(249.875) , 0)
    //         ),
    //         new Pose2d(Units.inchesToMeters(249.875), Units.inchesToMeters(0), new Rotation2d(0)), 
    //         trajectoryConfig);
        
    //     Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
    //         new Pose2d(Units.inchesToMeters(249.875), Units.inchesToMeters(-16), new Rotation2d(0)), 
    //         List.of(
    //         new Translation2d(0, Units.inchesToMeters(-16))), 
    //         new Pose2d(0, Units.inchesToMeters(-27.875), Rotation2d.fromDegrees(180)), 
    //         trajectoryConfig);
                
    //     Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(
    //         new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(-27.875), new Rotation2d(180)), 
    //         List.of(
    //         new Translation2d(0, Units.inchesToMeters(-47.625))), 
    //         new Pose2d(Units.inchesToMeters(80), Units.inchesToMeters(-47.625), Rotation2d.fromDegrees(0)), 
    //         trajectoryConfig);

    //     //Create PID Controllers
    //     PIDController xController = new PIDController(kPXController, 0, 0);
    //     PIDController yController = new PIDController(kPYController, 0, 0);
    //     ProfiledPIDController thetaController = new ProfiledPIDController(
    //         kPThetaController, 0, 0, kThetaControllerConstraints);
    //     thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    //     SwerveControllerCommand autoCommand = new SwerveControllerCommand(
    //         trajectory, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
    //         xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
    //     SwerveControllerCommand autoCommand2 = new SwerveControllerCommand(
    //         trajectory2, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
    //         xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
            
    //     SwerveControllerCommand autoCommand3 = new SwerveControllerCommand(
    //         trajectory3, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
    //         xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);

    //     return sequence(
    //         runOnce(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
    //         waitSeconds(3),
    //         autoCommand,
    //         runOnce(() -> swerveDrive.stopModules()),
    //         new TurnToAngle(0, swerveDrive),
    //         waitSeconds(1),
    //         autoCommand2, 
    //         new TurnToAngle(180, swerveDrive),
    //         waitSeconds(3),
    //         autoCommand3,
    //         new TheGreatBalancingAct(swerveDrive),
    //         runOnce(() -> swerveDrive.stopModules()),
    //         new TowSwerve(swerveDrive));
    // }     
}
