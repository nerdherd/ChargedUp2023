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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SwerveDrivetrain;
import static frc.robot.Constants.SwerveAutoConstants.*;

import java.util.List;

public class SwerveAutos {
    public static CommandBase translateBy(SwerveDrivetrain swerveDrive, double xTranslation, double yTranslation, double angle) {
        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
        // Create Actual Trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(),
            new Pose2d(xTranslation, yTranslation, new Rotation2d(angle)), 
            trajectoryConfig);
        
        //Create PID Controllers
        PIDController xController = new PIDController(kPXController, kIXController, kDXController);
        PIDController yController = new PIDController(kPYController,kIYController, kDYController);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand autoCommand = new SwerveControllerCommand(
            trajectory, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return Commands.sequence(
            autoCommand,
            Commands.runOnce(swerveDrive::stopModules, swerveDrive)
        );
    }

    public enum StartPosition {
        Left,
        Right,
        Middle
    }

    /**
     * Start with the front left swerve module aligned with the charging station's edge
     * in the x axis and around 8 inches to the right in the y axis
     * @param swerveDrive
     * @return
     */
    public static CommandBase twoPieceChargeAuto(SwerveDrivetrain swerveDrive, Arm arm, Claw claw, StartPosition position) {
        DriverStation.getAlliance();

        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
        // Create Actual Trajectory
        Trajectory fromStartToScore = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
            new Translation2d(0 , 0.25),
            new Translation2d(-.5 , 0.25)),
            new Pose2d(-.5, 0, new Rotation2d(0)), 
            trajectoryConfig);
        
        double pickupAngle = 0;
        double chargeYTranslation = 0;
        double pickupXDistance = 0;
        double pickupYDistance = 0;
        switch (position) {
            case Right:
                pickupAngle = -10;
                chargeYTranslation = -1.7;
                pickupXDistance = 4;
                pickupYDistance = -0.25;
                break;
            case Left:
                pickupAngle = 10;
                chargeYTranslation = 1.7;
                pickupXDistance = 4;
                pickupYDistance = 0.25;
                break;
            case Middle:
                pickupAngle = -20;
                chargeYTranslation = 0;
                pickupXDistance = 5; // TODO: Measure IRL
                pickupYDistance = -0.5;
                break;
        }
        
        
        Trajectory scoreToPickup = TrajectoryGenerator.generateTrajectory(
            new Pose2d(-.5, 0, new Rotation2d(0)), 
            List.of(
            new Translation2d(pickupXDistance, 0)), 
            new Pose2d(pickupXDistance, pickupYDistance, Rotation2d.fromDegrees(pickupAngle)), 
            trajectoryConfig);

        Trajectory pickupToScore = TrajectoryGenerator.generateTrajectory(
            new Pose2d(pickupXDistance, pickupYDistance, new Rotation2d(pickupAngle)), 
            List.of(
            new Translation2d(pickupXDistance, 0)), 
            new Pose2d(-0.5, 0, Rotation2d.fromDegrees(180)), 
            trajectoryConfig);
        
        Trajectory scoreToCharge = TrajectoryGenerator.generateTrajectory(
            new Pose2d(-0.5, 0, new Rotation2d(180)), 
            List.of(
            new Translation2d(-0.75, chargeYTranslation)), 
            new Pose2d(0, -1.5, Rotation2d.fromDegrees(0)), 
            trajectoryConfig);

        //Create PID Controllers
        PIDController xController = new PIDController(kPXController, kIXController, kDXController);
        PIDController yController = new PIDController(kPYController, kIYController, kDYController);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand startToScoreCommand = new SwerveControllerCommand(
            fromStartToScore, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        SwerveControllerCommand scoreToPickupCommand = new SwerveControllerCommand(
            scoreToPickup, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);

        SwerveControllerCommand pickupToScoreCommand = new SwerveControllerCommand(
            pickupToScore, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        SwerveControllerCommand scoreToChargeCommand = new SwerveControllerCommand(
            scoreToCharge, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return Commands.parallel(
            new RunCommand(() -> arm.moveArmMotionMagic()),
            new SequentialCommandGroup(
                            // Commands.runOnce(swerveDrive::zeroHeading),
                    new ParallelRaceGroup(
                        //new TurnToAngle(0, swerveDrive),
                        new WaitCommand(1)               
                    ),
                    Commands.runOnce(() -> SmartDashboard.putString("Stage", "Start")),
                    Commands.runOnce(() -> swerveDrive.resetOdometry(fromStartToScore.getInitialPose())),
                    // autoCommand,
                    // new TurnToAngle(180, swerveDrive),
                    Commands.runOnce(() -> swerveDrive.stopModules()),
                    new ParallelRaceGroup(
                        new SequentialCommandGroup(
                            new InstantCommand(() -> arm.setArmTargetTicks(ArmConstants.kArmScore)),
                            new WaitCommand(0.5),
                            Commands.waitUntil(arm.armAtTargetPosition)
                        ),
                        new WaitCommand(2)
                    ),
                    // new WaitCommand(1),

                    Commands.runOnce(() -> SmartDashboard.putString("Stage", "Score")),
                    arm.armExtend(),
                    new WaitCommand(1),

                    claw.clawOpen(),
                    new WaitCommand(1),
                    // claw.clawClose(),
                    arm.armStow(),
                    new WaitCommand(1),

                    new ParallelRaceGroup(
                        new SequentialCommandGroup(
                            new InstantCommand(() -> arm.setArmTargetTicks(ArmConstants.kArmStow)),
                            new WaitCommand(0.5),
                            Commands.waitUntil(arm.armAtTargetPosition)
                        ),
                        new WaitCommand(2)
                    ),
                    // new WaitCommand(1),
                    Commands.runOnce(() -> SmartDashboard.putString("Stage", "Stow")),
                    scoreToPickupCommand,
                    Commands.runOnce(() -> swerveDrive.stopModules()),
                    // arm.armExtend(),
                    // claw.clawOpen(),
                    // new WaitCommand(0.5),
                    new ParallelRaceGroup(
                        // new InstantCommand(() -> arm.setArmTargetTicks(ArmConstants.kArmGround)),
                        new SequentialCommandGroup(
                            new InstantCommand(() -> arm.setArmTargetTicks(ArmConstants.kArmGround)),
                            new WaitCommand(0.5),
                            Commands.waitUntil(arm.armAtTargetPosition)
                        ),
                        new WaitCommand(2)
                    ),
                    // claw.clawClose(),
                    // new WaitCommand(2),

                    Commands.runOnce(() -> SmartDashboard.putString("Stage", "Ground")),
                    claw.clawClose(),
                    new WaitCommand(1),
                    new ParallelRaceGroup(
                        new InstantCommand(() -> arm.setArmTargetTicks(ArmConstants.kArmStow)),

                        Commands.waitUntil(arm.armAtTargetPosition),
                        new WaitCommand(2)
                    ),
                    // new WaitCommand(1),

                    Commands.runOnce(() -> SmartDashboard.putString("Stage", "Stow 2")),
                    pickupToScoreCommand,
                    // Commands.runOnce(() -> swerveDrive.stopModules()),
                    // new WaitCommand(1),
                    // autoCommand4,
                    // Commands.runOnce(() -> swerveDrive.stopModules()),
                    // new WaitCommand(1),
                    // autoCommand5,
                    Commands.runOnce(() -> swerveDrive.stopModules()),
                    new ParallelRaceGroup(
                        new WaitCommand(0.5)
                        // new TurnToAngle(180, swerveDrive)                
                    ),
                    
                    new ParallelRaceGroup(
                        new InstantCommand(() -> arm.setArmTargetTicks(ArmConstants.kArmScore)),

                        Commands.waitUntil(arm.armAtTargetPosition),
                        new WaitCommand(2)
                    ),
                    
                    Commands.runOnce(() -> SmartDashboard.putString("Stage", "Score 2")),
                    arm.armExtend(),
                    new WaitCommand(1),

                    claw.clawOpen(),
                    new WaitCommand(1),
                    
                    arm.armStow(),
                    new WaitCommand(1),

                    new ParallelRaceGroup(
                        new InstantCommand(() -> arm.setArmTargetTicks(ArmConstants.kArmStow)),

                        Commands.waitUntil(arm.armAtTargetPosition),
                        new WaitCommand(2)
                    ),
                    new WaitCommand(1),

                    Commands.runOnce(() -> SmartDashboard.putString("Stage", "Stow 3")),
                    scoreToChargeCommand,
                    // new TheGreatBalancingAct(swerveDrive),
                    new TimedBalancingAct(swerveDrive, 0.5, SwerveAutoConstants.kPBalancingInitial, SwerveAutoConstants.kPBalancing),
                    Commands.runOnce(() -> swerveDrive.stopModules()))
                    
                );
    
        // return Commands.sequence(
            // new ParallelCommandGroup(
                
            // );
            // // Commands.runOnce(swerveDrive::zeroHeading),
            // new ParallelRaceGroup(
            //     //new TurnToAngle(0, swerveDrive),
            //     new WaitCommand(1)               
            // ),
            // Commands.runOnce(() -> SmartDashboard.putString("Stage", "Start")),
            // Commands.runOnce(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
            // // autoCommand,
            // // new TurnToAngle(180, swerveDrive),
            // Commands.runOnce(() -> swerveDrive.stopModules()),
            // new ParallelRaceGroup(
            //     arm.moveArmScore(),
            //     Commands.waitUntil(arm.armAtTargetPosition),
            //     new WaitCommand(2)
            // ),
            // new WaitCommand(1),

            // Commands.runOnce(() -> SmartDashboard.putString("Stage", "Score")),
            // arm.armExtend(),
            // //claw.clawOpen(),
            // new WaitCommand(1),
            // // claw.clawClose(),
            // arm.armStow(),
            // new ParallelRaceGroup(
            //     arm.moveArmStow(),
            //     Commands.waitUntil(arm.armAtTargetPosition),
            //     new WaitCommand(2)
            // ),
            // new WaitCommand(1),
            // Commands.runOnce(() -> SmartDashboard.putString("Stage", "Stow")),
            // autoCommand2, 
            // Commands.runOnce(() -> swerveDrive.stopModules()),
            // // arm.armExtend(),
            // // claw.clawOpen(),
            // // new WaitCommand(0.5),
            // new ParallelRaceGroup(
            //     arm.moveArmGround(),
            //     Commands.waitUntil(arm.armAtTargetPosition),
            //     new WaitCommand(2)
            // ),
            // new WaitCommand(1),

            // Commands.runOnce(() -> SmartDashboard.putString("Stage", "Ground")),
            // //claw.clawClose(),
            // new ParallelRaceGroup(
            //     arm.moveArmStow(),
            //     Commands.waitUntil(arm.armAtTargetPosition),
            //     new WaitCommand(2)
            // ),
            // new WaitCommand(1),

            // Commands.runOnce(() -> SmartDashboard.putString("Stage", "Stow 2")),
            // autoCommand3,
            // // Commands.runOnce(() -> swerveDrive.stopModules()),
            // // new WaitCommand(1),
            // // autoCommand4,
            // // Commands.runOnce(() -> swerveDrive.stopModules()),
            // // new WaitCommand(1),
            // // autoCommand5,
            // Commands.runOnce(() -> swerveDrive.stopModules()),
            // new ParallelRaceGroup(
            //     new WaitCommand(0.5)
            //     // new TurnToAngle(180, swerveDrive)                
            // ),
            // new ParallelRaceGroup(
            //     arm.moveArmScore(),
            //     Commands.waitUntil(arm.armAtTargetPosition),
            //     new WaitCommand(2)
            // ),
            
            // Commands.runOnce(() -> SmartDashboard.putString("Stage", "Score 2")),
            // arm.armExtend(),
            // // claw.clawOpen(),
            // new WaitCommand(1),
            
            // arm.armStow(),
            // new ParallelRaceGroup(
            //     arm.moveArmStow(),
            //     Commands.waitUntil(arm.armAtTargetPosition),
            //     new WaitCommand(2)
            // ),
            // new WaitCommand(1),

            // Commands.runOnce(() -> SmartDashboard.putString("Stage", "Stow 3")),
            // autoCommand6,
            // // new TheGreatBalancingAct(swerveDrive),
            // new TimedBalancingAct(swerveDrive, 0.5, SwerveAutoConstants.kPBalancingInitial, SwerveAutoConstants.kPBalancing),
            // Commands.runOnce(() -> swerveDrive.stopModules()));
    } 


    /**
     * Start with the front left swerve module aligned with the charging station's edge
     * in the x axis and around 8 inches to the right in the y axis
     * @param swerveDrive 
     * @return Command to reset odometry run auto to go onto charging station then run balancing auto
     */
    public static CommandBase chargeAuto(SwerveDrivetrain swerveDrive, StartPosition startPos, double waitTime, boolean goAround) {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
        
        double yTranslation = 0;
        double yOvershoot = 0;

        switch (startPos) {
            case Left:
                yTranslation = 1.75;
                yOvershoot = 2;
                break;
            case Right:
                yTranslation = -1.75;
                yOvershoot = -2;
                break;
            case Middle:
                break;
        }

        Trajectory trajectory;
        
        if (!goAround || startPos == StartPosition.Middle) {
            trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)), 
                List.of(
                    new Translation2d(0, yOvershoot)), 
                new Pose2d(1.5, yTranslation, Rotation2d.fromDegrees(0)), 
                trajectoryConfig);
        } else {
            trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)), 
                List.of(
                    new Translation2d(3, 0),
                    new Translation2d(3, yTranslation)), 
                new Pose2d(1.5, yTranslation, Rotation2d.fromDegrees(180)), 
                trajectoryConfig);
        }


        //Create PID Controllers
        PIDController xController = new PIDController(kPXController, kIXController, kDXController);
        PIDController yController = new PIDController(kPYController, kIYController, kDYController);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand autoCommand = new SwerveControllerCommand(
            trajectory, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return new SequentialCommandGroup(
            Commands.runOnce(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
            new WaitCommand(waitTime),
            autoCommand,
            new TimedBalancingAct(swerveDrive, 0.5, 
                SwerveAutoConstants.kPBalancingInitial, 
                SwerveAutoConstants.kPBalancing)
            // new TheGreatBalancingAct(swerveDrive),
            // new TowSwerve(swerveDrive)
        );
    }

     /**
     * Start with the front left swerve module aligned with the charging station's edge
     * in the x axis and around 8 inches to the right in the y axis
     * @param swerveDrive 
     * @return Command to reset odometry run auto to go onto charging station then run balancing auto
     */
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
        PIDController xController = new PIDController(kPXController, kIXController, kDXController);
        PIDController yController = new PIDController(kPYController, kIYController, kDYController);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);
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
