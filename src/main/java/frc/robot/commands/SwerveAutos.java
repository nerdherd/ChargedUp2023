package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.claw.MotorClaw;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.VROOOOM;
import frc.robot.subsystems.vision.VROOOOM.OBJECT_TYPE;
import frc.robot.subsystems.vision.VROOOOM.SCORE_POS;

import static frc.robot.Constants.SwerveAutoConstants.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.List;

public class SwerveAutos {
    public enum StartPosition {
        LEFT,
        RIGHT,
        MIDDLE
    }

    public static CommandBase driveBackwardAuto(SwerveDrivetrain swerveDrive) {
        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
        
        Trajectory driveForward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
            new Translation2d(-2, 0.02)), 
            new Pose2d(-4, 0.04, Rotation2d.fromDegrees(180)), 
            trajectoryConfig);
        
        //Create PID Controllers
        PIDController xController = new PIDController(kPXController, kIXController, kDXController);
        PIDController yController = new PIDController(kPYController, kIYController, kDYController);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        SwerveControllerCommand driveForwardCommand = new SwerveControllerCommand(
            driveForward, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return sequence(
            runOnce(() -> swerveDrive.setPoseMeters(driveForward.getInitialPose())),
            driveForwardCommand,
            runOnce(swerveDrive::stopModules)
        );
    }

    public static CommandBase driveBackwardLeftAuto(SwerveDrivetrain swerveDrive) {
        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
        
        Trajectory driveForward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
            new Translation2d(-2, 0)), 
            new Pose2d(-4, -1, Rotation2d.fromDegrees(180)), 
            trajectoryConfig);
        
        //Create PID Controllers
        PIDController xController = new PIDController(kPXController, kIXController, kDXController);
        PIDController yController = new PIDController(kPYController, kIYController, kDYController);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        SwerveControllerCommand driveForwardCommand = new SwerveControllerCommand(
            driveForward, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return sequence(
            runOnce(() -> swerveDrive.setPoseMeters(driveForward.getInitialPose())),
            driveForwardCommand,
            runOnce(swerveDrive::stopModules)
        );
    }

    /**
     * Start with the front left swerve module aligned with the charging station's edge
     * in the x axis and around 8 inches to the right in the y axis
     * @param swerveDrive
     * @return
     */
    public static CommandBase pickupAuto(SwerveDrivetrain swerveDrive, Arm arm, Elevator elevator, MotorClaw claw, StartPosition position, Alliance alliance, SCORE_POS scorePosition) {
        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
        
        double pickupXDistance = 0;
        double pickupYDistance = 0;

        if (alliance == Alliance.Red) {
            if (position == StartPosition.RIGHT) position = StartPosition.LEFT;
            if (position == StartPosition.LEFT) position = StartPosition.RIGHT;
        }


        int elevatorPos = ElevatorConstants.kElevatorScoreMid;
        switch (scorePosition) {
            case LOW:
                elevatorPos = ElevatorConstants.kElevatorStow;
                break;
            case MID:
                elevatorPos = ElevatorConstants.kElevatorScoreMid;
                break;
            case HIGH:
                elevatorPos = ElevatorConstants.kElevatorScoreHighCube;
                break;
        }

        final int elevatorPosFinal = elevatorPos;


        switch (position) {
            // Reversed because gyro starts reversed
            case RIGHT:
                pickupXDistance = -4.58;
                pickupYDistance = -0.08;
                break;
            case LEFT:
                pickupXDistance = -4.58;
                pickupYDistance = 0.17;
                break;
            case MIDDLE:
                pickupXDistance = 5; // TODO: Measure IRL
                pickupYDistance = -0.5;
                break;
        }
        
        if (alliance == Alliance.Red) {
            pickupYDistance *= -1;
        }
        
        Trajectory scoreToPickup = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
            new Translation2d(pickupXDistance / 2, pickupYDistance / 2)), 
            new Pose2d(pickupXDistance, pickupYDistance, Rotation2d.fromDegrees(180)), 
            trajectoryConfig);

        Trajectory pickupToScore = TrajectoryGenerator.generateTrajectory(
            new Pose2d(pickupXDistance, pickupYDistance, new Rotation2d(180)), 
            List.of(
            new Translation2d(pickupXDistance / 2, pickupYDistance / 2)), 
            new Pose2d(0, 0, Rotation2d.fromDegrees(180)), 
            trajectoryConfig);        

        //Create PID Controllers
        PIDController xController = new PIDController(kPXController, kIXController, kDXController);
        PIDController yController = new PIDController(kPYController, kIYController, kDYController);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        SwerveControllerCommand scoreToPickupCommand = new SwerveControllerCommand(
            scoreToPickup, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);

        SwerveControllerCommand pickupToScoreCommand = new SwerveControllerCommand(
            pickupToScore, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return deadline(
            sequence(
                parallel(
                    runOnce(() -> SmartDashboard.putString("Stage", "Start")),
                    runOnce(() -> swerveDrive.resetOdometry(scoreToPickup.getInitialPose())),
                    runOnce(() -> swerveDrive.stopModules())
                ),
                scoreToPickupCommand,
                runOnce(() -> swerveDrive.stopModules()),

                runOnce(() -> SmartDashboard.putString("Stage", "Ground")),
                deadline(
                    waitSeconds(2),
                    sequence(
                        runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup)),
                        waitSeconds(0.5),
                        waitUntil(arm.atTargetPosition)
                    )
                ),

                waitSeconds(0.75),
                claw.intake(),
                waitSeconds(0.25),

                runOnce(() -> SmartDashboard.putString("Stage", "Stow 2")),
                deadline(
                    waitSeconds(2),
                    runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                    waitUntil(arm.atTargetPosition)
                ),

                pickupToScoreCommand,

                runOnce(() -> swerveDrive.stopModules()),
                runOnce(() -> SmartDashboard.putString("Stage", "Score 2")),
                deadline(
                    waitSeconds(2),
                    sequence(
                        runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScore)),
                        waitSeconds(0.5),
                        waitUntil(arm.atTargetPosition)
                    ),
                    sequence(
                        runOnce(() -> elevator.setTargetTicks(elevatorPosFinal)),
                        waitSeconds(0.5),
                        waitUntil(elevator.atTargetPosition)
                    )
                ),

                waitSeconds(0.75),
                claw.outtake(),
                waitSeconds(0.25),

                runOnce(() -> SmartDashboard.putString("Stage", "Stow 3")),
                deadline(
                    waitSeconds(2),
                    sequence(
                        runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow)),
                        waitSeconds(0.5),
                        waitUntil(elevator.atTargetPosition)
                    ),
                    sequence(
                        runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                        waitSeconds(0.5),
                        waitUntil(arm.atTargetPosition)
                    ))
                ),
            run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
            run(() -> elevator.moveMotionMagic(arm.getArmAngle()))
            );
    } 

    public static CommandBase preloadAuto(Arm arm, Elevator elevator, MotorClaw claw, SCORE_POS scorePos) {
        int elevatorPos = ElevatorConstants.kElevatorScoreMid;
        int armPos = ArmConstants.kArmScoreCubeMid;

        switch (scorePos) {
            case LOW:
                elevatorPos = ElevatorConstants.kElevatorStow;
                armPos = ArmConstants.kArmScore; // Not real(ly accurate)
                break;
            case MID:
                elevatorPos = ElevatorConstants.kElevatorScoreMid;
                armPos = ArmConstants.kArmScore;
                break;
            case HIGH:
                elevatorPos = ElevatorConstants.kElevatorScoreHigh; //ElevatorConstants.kElevatorScoreHighCube; // Note: this is a bandaid
                armPos = ArmConstants.kArmScore;
                break;
        }

        final int elevatorPosFinal = elevatorPos;
        final int armPosFinal = armPos;

        return race(
            waitSeconds(5),
            sequence(
                claw.intake(),
                runOnce(() -> SmartDashboard.putString("Stage", "Score")),
                deadline(
                    waitSeconds(2),
                    sequence(
                        runOnce(() -> arm.setTargetTicks(armPosFinal)),
                        waitSeconds(0.5),
                        waitUntil(arm.atTargetPosition)
                    ),
                    sequence(
                        waitSeconds(0.5),
                        runOnce(() -> elevator.setTargetTicks(elevatorPosFinal)),
                        waitSeconds(0.5),
                        waitUntil(elevator.atTargetPosition)
                    )
                ),
                waitSeconds(0.5),
                // claw.outtake(),
                claw.setPower(.3),
                waitSeconds(0.5),
                claw.setPowerZero(),
                
                runOnce(() -> SmartDashboard.putString("Stage", "Stow")),
                deadline(
                    waitSeconds(0.5),
                    sequence(
                        runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow)),
                        waitSeconds(0.5),
                        waitUntil(elevator.atTargetPosition)
                    ),
                    sequence(
                        runOnce(() -> arm.setTargetTicks(ArmConstants.kArmSubstation)),
                        waitSeconds(0.5),
                        waitUntil(arm.atTargetPosition)
                    )
                )
            ),
            run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
            run(() -> elevator.moveMotionMagic(arm.getArmAngle()))
        );
    }

    public static CommandBase pickupChargeAuto(SwerveDrivetrain swerveDrive, Arm arm, Elevator elevator, MotorClaw claw, StartPosition position, Alliance alliance, SCORE_POS scorePos) {
        return sequence(

            pickupAuto(swerveDrive, arm, elevator, claw, position, alliance, scorePos),
            chargeAuto(swerveDrive, position, alliance, 0, false));
    }

    public static CommandBase pickupBackwardAuto(SwerveDrivetrain swerveDrive, Arm arm, Elevator elevator, MotorClaw claw, StartPosition position, SCORE_POS scorePos, Alliance alliance) {
        return sequence(
            pickupAuto(swerveDrive, arm, elevator, claw, position, alliance, scorePos),
            driveBackwardAuto(swerveDrive)
        );
    }

    public static CommandBase preloadChargeAuto(SwerveDrivetrain swerveDrive, Arm arm, Elevator elevator, MotorClaw claw, StartPosition startPos, SCORE_POS scorePos, double waitTime, boolean goAround, Alliance alliance) {
        return sequence(
            preloadAuto(arm, elevator, claw, scorePos),
            deadline(
                chargeAuto(swerveDrive, startPos, alliance, waitTime, goAround),
                run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
                run(() -> elevator.moveMotionMagic(arm.getArmAngle()))
            )
        );
    }

    public static CommandBase preloadBackwardAuto(SwerveDrivetrain swerveDrive, Arm arm, Elevator elevator, MotorClaw claw, StartPosition startPos, SCORE_POS scorePos, Alliance alliance) {
        return sequence(
            preloadAuto(arm, elevator, claw, scorePos),
            driveBackwardAuto(swerveDrive)
        );
    }

    
    public static CommandBase preloadBackwardLeftAuto(SwerveDrivetrain swerveDrive, Arm arm, Elevator elevator, MotorClaw claw, StartPosition startPos, SCORE_POS scorePos, Alliance alliance) {
        return sequence(
            preloadAuto(arm, elevator, claw, scorePos),
            driveBackwardLeftAuto(swerveDrive)
        );
    }

    public static CommandBase twoPieceAuto(SwerveDrivetrain swerveDrive, Arm arm, Elevator elevator, MotorClaw claw, StartPosition startPos, SCORE_POS scorePos, Alliance alliance) {
        return sequence(
            preloadAuto(arm, elevator, claw, scorePos),
            pickupAuto(swerveDrive, arm, elevator, claw, startPos, alliance, scorePos)
        );
    }

    public static CommandBase twoPieceChargeAuto(SwerveDrivetrain swerveDrive, Arm arm, Elevator elevator, MotorClaw claw, StartPosition startPos, SCORE_POS scorePos, double waitTime, boolean goAround, Alliance alliance) {
        return sequence(
            preloadAuto(arm, elevator, claw, scorePos),
            pickupAuto(swerveDrive, arm, elevator, claw, startPos, alliance, scorePos),
            chargeAuto(swerveDrive, startPos, alliance, waitTime, goAround)
        );
    }

    public static CommandBase twoPieceBackwardAuto(SwerveDrivetrain swerveDrive, Arm arm, Elevator elevator, MotorClaw claw, StartPosition startPos, SCORE_POS scorePos, Alliance alliance) {
        return sequence(
            preloadAuto(arm, elevator, claw, scorePos),
            pickupAuto(swerveDrive, arm, elevator, claw, startPos, alliance, scorePos),
            driveBackwardAuto(swerveDrive)
        );
    }
    
    /**
     * Start with the swerve drive facing the driver at either the rightmost cone grid, the leftmost cone grid, or directly in front of the charging station (middle)
     * @param swerveDrive 
     * @return Command to reset odometry run auto to go onto charging station then run balancing auto
     */
    public static CommandBase chargeAuto(SwerveDrivetrain swerveDrive, StartPosition startPos, Alliance alliance, double waitTime, boolean goAround) {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
        
        double yTranslation = 0;
        double yOvershoot = 0;

        switch (startPos) {
            case LEFT:
                yTranslation = -1.75;
                yOvershoot = -1.75;
                break;
            case RIGHT:
                yTranslation = 1.75;
                yOvershoot = 1.75;
                break;
            case MIDDLE:
                break;
        }

        if (alliance == Alliance.Red) {
            yTranslation *= -1;
            yOvershoot *= -1;
        }

        Trajectory trajectory;
        
        if (!goAround) {
            trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(-0.125, 0, new Rotation2d(0)), 
                List.of(
                    new Translation2d(-0.25, 0),
                    new Translation2d(-0.25, yOvershoot)), 
                new Pose2d(-2.7, yTranslation - 0.01, Rotation2d.fromDegrees(0)), // -2
                trajectoryConfig);
        } else {
            trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)), 
                List.of(
                    new Translation2d(-3.5, yTranslation / 4),
                    new Translation2d(-3.5, yTranslation + 0.01)), 
                new Pose2d(-2, yTranslation - 0.01, Rotation2d.fromDegrees(0)), 
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
        
        return sequence(
            runOnce(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
            waitSeconds(waitTime),
            autoCommand,
            new TheGreatBalancingAct(swerveDrive)
            // new TowSwerve(swerveDrive)
        );
    }
    
     /**
     * Start with the front left swerve module aligned with the charging station's edge
     * in the x axis and around 8 inches to the right in the y axis
     * 
     * Use as a backup only
     * 
     * @param swerveDrive 
     * @return Command to reset odometry run auto to go onto charging station then run balancing auto
     */
    public static CommandBase backupChargeAuto(SwerveDrivetrain swerveDrive) {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
        
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                new Translation2d(-0.75, 0), // -0.75
                new Translation2d(-0.75, -1.25)), 
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
        
        return sequence(
            runOnce(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
            deadline(
                waitSeconds(3),
                autoCommand
            ),
            new TheGreatBalancingAct(swerveDrive)
            // new TowSwerve(swerveDrive)
        );
    }
}
