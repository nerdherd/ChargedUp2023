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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MotorClaw;
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

    /**
     * Start with the front left swerve module aligned with the charging station's edge
     * in the x axis and around 8 inches to the right in the y axis
     * @param swerveDrive
     * @return
     */
    public static CommandBase onePieceChargeAuto(SwerveDrivetrain swerveDrive, Arm arm, Elevator elevator, MotorClaw claw, StartPosition position, Alliance alliance) {
        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
        
        double pickupAngle = 0;
        double chargeYTranslation = 0;
        double pickupXDistance = 0;
        double pickupYDistance = 0;

        if (alliance == Alliance.Red) {
            if (position == StartPosition.RIGHT) position = StartPosition.LEFT;
            if (position == StartPosition.LEFT) position = StartPosition.RIGHT;
        }

        switch (position) {
            case RIGHT:
                pickupAngle = -10;
                chargeYTranslation = -1.7;
                pickupXDistance = 4;
                pickupYDistance = -0.25;
                break;
            case LEFT:
                pickupAngle = 10;
                chargeYTranslation = 1.7;
                pickupXDistance = 4;
                pickupYDistance = 0.25;
                break;
            case MIDDLE:
                pickupAngle = -20;
                chargeYTranslation = 0;
                pickupXDistance = 5; // TODO: Measure IRL
                pickupYDistance = -0.5;
                break;
        }
        
        if (alliance == Alliance.Red) {
            chargeYTranslation *= -1;
            pickupYDistance *= -1;
            pickupAngle *= -1;
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
        
        SwerveControllerCommand scoreToPickupCommand = new SwerveControllerCommand(
            scoreToPickup, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);

        SwerveControllerCommand pickupToScoreCommand = new SwerveControllerCommand(
            pickupToScore, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        SwerveControllerCommand scoreToChargeCommand = new SwerveControllerCommand(
            scoreToCharge, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return parallel(
            run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
            run(() -> elevator.moveMotionMagic(arm.getArmAngle())),
            sequence(
                parallel(
                    runOnce(() -> SmartDashboard.putString("Stage", "Start")),
                    runOnce(() -> swerveDrive.resetOdometry(scoreToPickup.getInitialPose())),
                    runOnce(() -> swerveDrive.stopModules())
                ),

                runOnce(() -> SmartDashboard.putString("Stage", "Score")),
                deadline(
                    waitSeconds(2),
                    sequence(
                        runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScore)),
                        waitSeconds(0.5),
                        waitUntil(arm.atTargetPosition)
                    ),
                    sequence(
                        runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorScoreHigh)),
                        waitSeconds(0.5),
                        waitUntil(elevator.atTargetPosition)
                    )
                ),

                claw.outtake(),
                
                runOnce(() -> SmartDashboard.putString("Stage", "Stow")),
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
                    )
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

                claw.intake(),

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
                        runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorScoreHigh)),
                        waitSeconds(0.5),
                        waitUntil(elevator.atTargetPosition)
                    )
                ),
                claw.outtake(),

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
                    )
                ),

                scoreToChargeCommand,
                // new TheGreatBalancingAct(swerveDrive),
                new TimedBalancingAct(swerveDrive, 0.5, SwerveAutoConstants.kPBalancingInitial, SwerveAutoConstants.kPBalancing),
                runOnce(() -> swerveDrive.stopModules()))
            );
    } 

    public static CommandBase preloadChargeAuto(SwerveDrivetrain swerveDrive, Arm arm, Elevator elevator, MotorClaw claw, StartPosition startPos, SCORE_POS scorePos, double waitTime, boolean goAround) {
        return parallel(
            run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
            run(() -> elevator.moveMotionMagic(arm.getArmAngle())),
            sequence(
                runOnce(() -> SmartDashboard.putString("Stage", "Score")),
                deadline(
                    waitSeconds(2),
                    sequence(
                        runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScore)),
                        waitSeconds(0.5),
                        waitUntil(arm.atTargetPosition)
                    ),
                    sequence(
                        runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorScoreHigh)),
                        waitSeconds(0.5),
                        waitUntil(elevator.atTargetPosition)
                    )
                ),
                claw.outtake(),
                runOnce(() -> SmartDashboard.putString("Stage", "Stow")),
                deadline(
                    waitSeconds(2),
                    sequence(
                        runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                        waitSeconds(0.5),
                        waitUntil(arm.atTargetPosition)
                    ),
                    sequence(
                        runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow)),
                        waitSeconds(0.5),
                        waitUntil(elevator.atTargetPosition)
                    )
                ),
                chargeAuto(swerveDrive, startPos, waitTime, goAround)
            )
        );
    }

    public static CommandBase visionPreloadChargeAuto(VROOOOM vision, SwerveDrivetrain swerveDrive, Arm arm, Elevator elevator, MotorClaw claw, StartPosition startPos, SCORE_POS scorePos, double waitTime, boolean goAround) {
        return parallel(
            run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
            run(() -> elevator.moveMotionMagic(arm.getArmAngle())),
            sequence(
                // parallel(
                //     //runOnce(() -> vision.updateCurrentGameObject(OBJECT_TYPE.CONE)),
                //     runOnce(() -> vision.updateCurrentHeight(SCORE_POS.MID))
                // ),
                vision.VisionScore(OBJECT_TYPE.CONE, SCORE_POS.MID),
                chargeAuto(swerveDrive, startPos, waitTime, goAround)
            )
        );
    }

    public static CommandBase preloadBackup(SwerveDrivetrain swerveDrive, Arm arm, Elevator elevator, MotorClaw claw, StartPosition startPos, SCORE_POS scorePos, double waitTime, boolean goAround) {
        return parallel(
            run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
            run(() -> elevator.moveMotionMagic(arm.getArmAngle())),
            sequence(
                runOnce(() -> SmartDashboard.putString("Stage", "Score")),
                deadline(
                    waitSeconds(2),
                    sequence(
                        runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScore)),
                        waitSeconds(0.5),
                        waitUntil(arm.atTargetPosition)
                    ),
                    sequence(
                        runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorScoreHigh)),
                        waitSeconds(0.5),
                        waitUntil(elevator.atTargetPosition)
                    )
                ),
                claw.outtake(),
                runOnce(() -> SmartDashboard.putString("Stage", "Stow")),
                deadline(
                    waitSeconds(2),
                    sequence(
                        runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                        waitSeconds(0.5),
                        waitUntil(arm.atTargetPosition)
                    ),
                    sequence(
                        runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow)),
                        waitSeconds(0.5),
                        waitUntil(elevator.atTargetPosition)
                    )
                ),
                backupChargeAuto(swerveDrive)
            )
        );
    }

    public static CommandBase backupVisionPreloadChargeAuto(VROOOOM vision, SwerveDrivetrain swerveDrive, Arm arm, Elevator elevator, MotorClaw claw, StartPosition startPos, SCORE_POS scorePos, double waitTime, boolean goAround) {
        return parallel(
            run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
            run(() -> elevator.moveMotionMagic(arm.getArmAngle())),
            sequence(
                // parallel(
                //     //runOnce(() -> vision.updateCurrentGameObject(OBJECT_TYPE.CONE)),
                //     runOnce(() -> vision.updateCurrentHeight(SCORE_POS.MID))
                // ),
                vision.VisionScore(OBJECT_TYPE.CONE, SCORE_POS.MID),
                backupChargeAuto(swerveDrive)
            )
        );
    }

    public static CommandBase chargeAuto(SwerveDrivetrain swerveDrive, StartPosition startPos, double waitTime, boolean goAround) {
        return chargeAuto(swerveDrive, startPos, DriverStation.getAlliance(), waitTime, goAround);
    }

    /**
     * Start with the front left swerve module aligned with the charging station's edge
     * in the x axis and around 8 inches to the right in the y axis
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
                yTranslation = 1.75;
                yOvershoot = 1.75;
                break;
            case RIGHT:
                yTranslation = -1.75;
                yOvershoot = -1.75;
                break;
            case MIDDLE:
                break;
        }

        if (alliance == Alliance.Red) {
            yTranslation *= -1;
            yOvershoot *= -1;
        }

        Trajectory trajectory;
        
        if (!goAround || startPos == StartPosition.MIDDLE) {
            trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(180)), 
                List.of(
                    new Translation2d(0.25, yOvershoot)), 
                new Pose2d(1.5, yTranslation, Rotation2d.fromDegrees(180)), 
                trajectoryConfig);
        } else {
            trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(180)), 
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
        
        return sequence(
            runOnce(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
            waitSeconds(waitTime),
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
                new Translation2d(-0.75, 0),
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
            new TimedBalancingAct(swerveDrive, 0.25, 
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
     * @return
     */
    public static CommandBase backupTwoPieceChargeAuto(SwerveDrivetrain swerveDrive, Arm arm, Elevator elevator, MotorClaw claw) {
        
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
        
        Trajectory trajectory6 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(-0.5, 0, new Rotation2d(180)), 
            List.of(
            new Translation2d(-0.75, -1.7)), 
            new Pose2d(0, -1.5, Rotation2d.fromDegrees(0)), 
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
        
        SwerveControllerCommand autoCommand6 = new SwerveControllerCommand(
            trajectory6, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return sequence(
            parallel(
                runOnce(() -> SmartDashboard.putString("Stage", "Start")),
                runOnce(() -> swerveDrive.resetOdometry(trajectory.getInitialPose()))
            ),
            autoCommand,
            runOnce(() -> swerveDrive.stopModules()),
            
            runOnce(() -> SmartDashboard.putString("Stage", "Score")),
            deadline(
                waitSeconds(2),
                sequence(
                    runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScore)),
                    waitSeconds(0.5),
                    waitUntil(arm.atTargetPosition)
                ),
                sequence(
                    runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorScoreHigh)),
                    waitSeconds(0.5),
                    waitUntil(elevator.atTargetPosition)
                )
            ),
            claw.outtake(),

            runOnce(() -> SmartDashboard.putString("Stage", "Stow")),
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
                )
            ),

            autoCommand2, 
            runOnce(() -> SmartDashboard.putString("Stage", "Ground")),
            deadline(
                waitSeconds(2),
                sequence(
                    runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup)),
                    waitSeconds(0.5),
                    waitUntil(arm.atTargetPosition)
                )
            ),

            claw.intake(),

            runOnce(() -> SmartDashboard.putString("Stage", "Stow 2")),
            deadline(
                waitSeconds(2),
                runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                waitUntil(arm.atTargetPosition)
            ),

            autoCommand3,
            runOnce(() -> swerveDrive.stopModules()),
            
            deadline(
                waitSeconds(2),
                sequence(
                    runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScore)),
                    waitSeconds(0.5),
                    waitUntil(arm.atTargetPosition)
                ),
                sequence(
                    runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorScoreHigh)),
                    waitSeconds(0.5),
                    waitUntil(elevator.atTargetPosition)
                )
            ),

            claw.outtake(),

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
                )
            ),

            autoCommand6,
            // new TheGreatBalancingAct(swerveDrive),
            new TimedBalancingAct(swerveDrive, 0.5, SwerveAutoConstants.kPBalancingInitial, SwerveAutoConstants.kPBalancing),
            runOnce(() -> swerveDrive.stopModules()));
    } 
}
