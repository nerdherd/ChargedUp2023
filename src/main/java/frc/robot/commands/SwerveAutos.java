package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.commands.TheGreatBalancingAct;
import static frc.robot.Constants.SwerveAutoConstants.*;

import java.util.List;

public class SwerveAutos {
    /**
     * Start with the front left swerve module aligned with the charging station's edge
     * in the x axis and around 8 inches to the right in the y axis
     * @param swerveDrive
     * @return
     */
    public static CommandBase testAuto(SwerveDrivetrain swerveDrive) {
        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
        // Create Actual Trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
            new Translation2d(0 , 0.25),
            new Translation2d(-.5 , 0.25)),
            new Pose2d(-.5, 0, new Rotation2d(0)), 
            trajectoryConfig);
        
        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(-.5, 0, new Rotation2d(0)), 
            List.of(
            new Translation2d(4, 0)), 
            new Pose2d(4, -0.25, Rotation2d.fromDegrees(-10)), 
            trajectoryConfig);
        
        Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(4, -0.25, new Rotation2d(-10)), 
            List.of(
            new Translation2d(4, 0)), 
            new Pose2d(-0.5, 0, Rotation2d.fromDegrees(180)), 
            trajectoryConfig);
        
        Trajectory trajectory4 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(-0.5, 0, new Rotation2d(180)), 
            List.of(
            new Translation2d(4, 0)), 
            new Pose2d(4, -1.5, Rotation2d.fromDegrees(-20)), 
            trajectoryConfig);
        
        Trajectory trajectory5 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(4, -1.5, new Rotation2d(-20)), 
            List.of(
            new Translation2d(4, -0.25)), 
            new Pose2d(-0.1, 0, Rotation2d.fromDegrees(180)), 
            trajectoryConfig);
        
        Trajectory trajectory6 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(-0.1, 0, new Rotation2d(180)), 
            List.of(
            new Translation2d(0, -2)), 
            new Pose2d(2, -1.75, Rotation2d.fromDegrees(0)), 
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
        
        SwerveControllerCommand autoCommand2 = new SwerveControllerCommand(
            trajectory2, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);

        SwerveControllerCommand autoCommand3 = new SwerveControllerCommand(
            trajectory3, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        SwerveControllerCommand autoCommand4 = new SwerveControllerCommand(
            trajectory4, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        SwerveControllerCommand autoCommand5 = new SwerveControllerCommand(
            trajectory5, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        SwerveControllerCommand autoCommand6 = new SwerveControllerCommand(
            trajectory6, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return Commands.sequence(
            // Commands.runOnce(swerveDrive::zeroHeading),
            new ParallelRaceGroup(
                new TurnToAngle(0, swerveDrive),
                new WaitCommand(1)               
            ),
            Commands.runOnce(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
            autoCommand,
            new TurnToAngle(180, swerveDrive),
            Commands.runOnce(() -> swerveDrive.stopModules()),
            new WaitCommand(1.5),
            autoCommand2, 
            Commands.runOnce(() -> swerveDrive.stopModules()),
            new WaitCommand(2),
            autoCommand3,
            Commands.runOnce(() -> swerveDrive.stopModules()),
            new WaitCommand(2),
            autoCommand4,
            Commands.runOnce(() -> swerveDrive.stopModules()),
            new WaitCommand(2),
            autoCommand5,
            Commands.runOnce(() -> swerveDrive.stopModules()),
            new WaitCommand(2),
            autoCommand6,
            new TheGreatBalancingAct(swerveDrive),
            Commands.runOnce(() -> swerveDrive.stopModules()));
    } 

    public static CommandBase chargeAuto(SwerveDrivetrain swerveDrive) {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
        
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                new Translation2d(-0.5, 0),
                new Translation2d(-0.5, -2)), 
            new Pose2d(1, -1.75, Rotation2d.fromDegrees(0)), 
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
        
        return new SequentialCommandGroup(
            Commands.runOnce(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
            autoCommand,
            new TimedBalancingAct(swerveDrive, 0.5, 
                SwerveAutoConstants.kPBalancingInitial, 
                SwerveAutoConstants.kPBalancing)
            // new TheGreatBalancingAct(swerveDrive),
            // new TowSwerve(swerveDrive)
        );
    }

    public static CommandBase hardCarryAuto(SwerveDrivetrain swerveDrive) {
        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
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
        PIDController xController = new PIDController(kPXController, 0, 0);
        PIDController yController = new PIDController(kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, 0, 0, kThetaControllerConstraints);
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

        return Commands.sequence(
            new WaitCommand(2),
            Commands.runOnce(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
            new TurnToAngle(180, swerveDrive),
            new WaitCommand(3),
            autoCommand,
            Commands.runOnce(() -> swerveDrive.stopModules()),
            new TurnToAngle(0, swerveDrive),
            new WaitCommand(1),
            autoCommand2, 
            new TurnToAngle(180, swerveDrive),
            new WaitCommand(3),
            autoCommand3,
            new TheGreatBalancingAct(swerveDrive),
            Commands.runOnce(() -> swerveDrive.stopModules()),
            new TowSwerve(swerveDrive));
    }

    public static CommandBase vendingMachine(SwerveDrivetrain swerveDrive) {
        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
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
        PIDController xController = new PIDController(kPXController, 0, 0);
        PIDController yController = new PIDController(kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, 0, 0, kThetaControllerConstraints);
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

        return Commands.sequence(
            new WaitCommand(4),
            Commands.runOnce(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
            new TurnToAngle(180, swerveDrive),
            new WaitCommand(3),
            autoCommand,
            Commands.runOnce(() -> swerveDrive.stopModules()),
            new TurnToAngle(0, swerveDrive),
            new WaitCommand(1),
            autoCommand2, 
            new TurnToAngle(180, swerveDrive),
            new WaitCommand(3),
            autoCommand3,
            new TheGreatBalancingAct(swerveDrive),
            Commands.runOnce(() -> swerveDrive.stopModules()),
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

    //     return Commands.sequence(
    //         Commands.runOnce(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
    //         new WaitCommand(3),
    //         autoCommand,
    //         Commands.runOnce(() -> swerveDrive.stopModules()),
    //         new TurnToAngle(0, swerveDrive),
    //         new WaitCommand(1),
    //         autoCommand2, 
    //         new TurnToAngle(180, swerveDrive),
    //         new WaitCommand(3),
    //         autoCommand3,
    //         new TheGreatBalancingAct(swerveDrive),
    //         Commands.runOnce(() -> swerveDrive.stopModules()),
    //         new TowSwerve(swerveDrive));
    // } 
}
