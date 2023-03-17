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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.SwerveAutos.StartPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.claw.MotorClaw;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.VROOOOM;
import frc.robot.subsystems.vision.VROOOOM.OBJECT_TYPE;
import frc.robot.subsystems.vision.VROOOOM.SCORE_POS;

import static frc.robot.Constants.SwerveAutoConstants.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

// Trajectory must stop ~32 inches in front of the cone
// Trajectory can stop anywhere in front of the tape/tag as long as the robot is in front of the charging station
// All score positions are assumed to be high for this file right now
// Positive Y = left, positive X = towards drivers, rotation positive is clockwise

public class VisionAutos {
    
    /**
     * You should not be calling this function directly (nothing will break if you do, 
     * but it is meant to be paired with vision auto choosing). Instead, call visionAutoChoose below.
     * 
     * @return Auto Command
     */
    public static CommandBase visionPreloadPickupScoreCharge(SwerveDrivetrain swerveDrive, VROOOOM vision, Arm arm, Elevator elevator, MotorClaw claw, 
        Alliance alliance, StartPosition position, SCORE_POS scorePosition) {
        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            SwerveAutoConstants.kMaxSpeedMetersPerSecond, SwerveAutoConstants.kMaxAccelerationMetersPerSecondSquared);
        
        double pickupXDistance = 0;
        double pickupYDistance = 0;
        double pickupRotation = 0;

        double avoidCollisionYOffset = 0; // So we don't bump into the charging station

        // Position when scoring the object we picked up
        double scoreXDistance = 0;
        double scoreYDistance = 0;

        if (alliance == Alliance.Red) {
            if (position == StartPosition.RIGHT) position = StartPosition.LEFT;
            if (position == StartPosition.LEFT) position = StartPosition.RIGHT;
        }

        switch (position) {
            // Reversed because gyro starts reversed
            case RIGHT:
                pickupXDistance = -3.73;
                pickupYDistance = 0.93;
                pickupRotation = -175;
                avoidCollisionYOffset = 0.28;
                scoreXDistance = -0.26;
                scoreYDistance = 0.5;
                break;
            case LEFT:
                pickupXDistance = -3.73;
                pickupYDistance = -0.93;
                pickupRotation = 175; // Tested at Da Vinci, not accurate likely because of limelight placement
                avoidCollisionYOffset = -0.28; // Was 0.75 when tested at Da Vinci with only 1 waypoint, but now we're using 2
                scoreXDistance = -0.26;
                scoreYDistance = -0.5;
                break;
            case MIDDLE:
                pickupXDistance = -5; // TODO: Measure IRL
                pickupYDistance = -0.3;
                pickupRotation = -165;
                scoreXDistance = -0.26;
                scoreYDistance = 0.5;
                break;
        }
        
        // Negate all measurements related to the y-axis because we are on the opposite side of the field
        if (alliance == Alliance.Red) {
            if (position != StartPosition.MIDDLE) {
                pickupYDistance *= -1;
                avoidCollisionYOffset *= -1;
                scoreYDistance *= -1;
                pickupRotation *= -1;
            }
        }
        
        Trajectory scoreToPickup = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
            new Translation2d(pickupXDistance * 0.25, avoidCollisionYOffset),
            new Translation2d(pickupXDistance * 0.75, avoidCollisionYOffset)),
            new Pose2d(pickupXDistance, pickupYDistance, Rotation2d.fromDegrees(pickupRotation)),
            trajectoryConfig);

        // Trajectory with an inital pose
        // Trajectory pickupToScore = TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(pickupXDistance, pickupYDistance, new Rotation2d(pickupRotation)), 
        //     List.of(
        //     new Translation2d(pickupXDistance * 0.75, yOffset),
        //     new Translation2d(pickupXDistance * 0.25, yOffset)),
        //     new Pose2d(scoreXDistance, scoreYDistance, Rotation2d.fromDegrees(0)),
        //     trajectoryConfig);

        // Trajectory without an intial pose so we don't do unnecessary movement after vision pickup
        Trajectory pickupToScore = TrajectoryGenerator.generateTrajectory(
            List.of(
            new Pose2d(pickupXDistance * 0.75, avoidCollisionYOffset, Rotation2d.fromDegrees(pickupRotation * 0.75)),
            new Pose2d(pickupXDistance * 0.25, avoidCollisionYOffset, Rotation2d.fromDegrees(pickupRotation * 0.25)),
            new Pose2d(scoreXDistance, scoreYDistance, Rotation2d.fromDegrees(0)))
            , trajectoryConfig);

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
        
        final StartPosition startPositionFinal = position;
        final Alliance allianceFinal = alliance;

        return race(
            sequence(
                
                // ======== Drop Preload Start ========
                Commands.parallel(
                    runOnce(() -> SmartDashboard.putString("Stage", "Start")),
                    runOnce(() -> swerveDrive.resetOdometry(scoreToPickup.getInitialPose())),
                    runOnce(() -> swerveDrive.stopModules())
                ),

                // Move arm and elevator, arm is moved 0.5 seconds after the elevator to prevent power chain from getting caught
                Commands.race(
                    Commands.waitSeconds(5), // Timeout
                    Commands.sequence(
                        Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScore)),
                        Commands.waitSeconds(0.5),

                        Commands.parallel( // End when target positions reached
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorScoreMid))
                        )
                    )
                ),
                waitSeconds(0.25),

                // Open claw/eject piece with rollers
                claw.setPower(0.5),

                // Wait to outtake
                Commands.waitSeconds(.5),

                // Close claw/stop rollers
                claw.setPower(0),

                // Parallel driving to pickup position and moving arm/elev to ready-to-pickup position
                Commands.deadline(
                    scoreToPickupCommand,

                    // Move arm to ready-to-pickup position
                    Commands.sequence(
                        Commands.deadline(
                            Commands.waitSeconds(2),
                            Commands.parallel( // End command once both arm and elevator have reached their target position
                                Commands.waitUntil(arm.atTargetPosition),
                                Commands.waitUntil(elevator.atTargetPosition),
                                Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                                Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow))
                            )
                        ),
                        Commands.deadline(
                            Commands.waitSeconds(2),
                            Commands.parallel( // End command once both arm and elevator have reached their target position
                                Commands.waitUntil(arm.atTargetPosition),
                                Commands.waitUntil(elevator.atTargetPosition),
                                Commands.runOnce(() -> arm.setTargetTicks(-328500 + 7950)),
                                Commands.runOnce(() -> elevator.setTargetTicks(-160000))
                            )
                        )
                    )
                ),
                runOnce(() -> swerveDrive.stopModules()),

                // Arm is moved to pick up cube, ends with arm/elev extended and cube in the claw
                vision.VisionPickupGroundNoArm(OBJECT_TYPE.CUBE),

                // Drive to score in parallel with arm moving to score position and elevator stowing
                Commands.parallel(
                    pickupToScoreCommand,

                    Commands.deadline(
                        Commands.waitSeconds(2),
                        Commands.parallel( // End command once both arm and elevator have reached their target position
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScore)),
                            Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorScoreMid))
                        )
                    )
                ),

                parallel(
                    runOnce(() -> swerveDrive.stopModules()),
                    runOnce(() -> SmartDashboard.putString("Stage", "Score 2"))
                ),

                // Drive to target and score, ends with arm/elev fully extended to score
                vision.VisionScoreNoArm(OBJECT_TYPE.CUBE, SCORE_POS.MID),
                    
                // Stow the elevator, move arm to substation pos, and charge in parallel
                Commands.parallel(
                    Commands.deadline(
                        Commands.waitSeconds(2),
                        Commands.parallel( // End command once both arm and elevator have reached their target position
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                            Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow))
                        )
                    ),

                    VisionAutos.visionChargeAuto(swerveDrive, startPositionFinal, allianceFinal, 0, false)
                )
            ),
            
            run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
            run(() -> elevator.moveMotionMagic(arm.getArmAngle()))
        );
    }

    public static CommandBase visionPreloadPickupScore(SwerveDrivetrain swerveDrive, VROOOOM vision, Arm arm, Elevator elevator, MotorClaw claw, 
        Alliance alliance, StartPosition position, SCORE_POS scorePosition) {
        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            SwerveAutoConstants.kMaxSpeedMetersPerSecond, SwerveAutoConstants.kMaxAccelerationMetersPerSecondSquared);
        
        double pickupXDistance = 0;
        double pickupYDistance = 0;
        double pickupRotation = 0;

        double avoidCollisionYOffset = 0; // So we don't bump into the charging station

        // Position when scoring the object we picked up
        double scoreXDistance = 0;
        double scoreYDistance = 0;

        if (alliance == Alliance.Red) {
            if (position == StartPosition.RIGHT) position = StartPosition.LEFT;
            if (position == StartPosition.LEFT) position = StartPosition.RIGHT;
        }

        switch (position) {
            // Reversed because gyro starts reversed
            case RIGHT:
                pickupXDistance = -3.73;
                pickupYDistance = 0.93;
                pickupRotation = -175;
                scoreYDistance = 0.5;
                avoidCollisionYOffset = 0.25;
                break;
            case LEFT:
                pickupXDistance = -3.73;
                pickupYDistance = -0.93;
                scoreYDistance = -0.5;
                pickupRotation = 175; // Tested at Da Vinci, not accurate likely because of limelight placement
                avoidCollisionYOffset = -0.25; // Was 0.75 when tested at Da Vinci with only 1 waypoint, but now we're using 2
                break;
            case MIDDLE:
                pickupXDistance = -5; // TODO: Measure IRL
                pickupYDistance = -0.3;
                pickupRotation = -165;
                break;
        }

        // Negate all measurements related to the y-axis because we are on the opposite side of the field
        if (alliance == Alliance.Red) {
            if (position != StartPosition.MIDDLE) {
                pickupYDistance *= -1;
                avoidCollisionYOffset *= -1;
                pickupRotation *= -1;
            }
        }

        Trajectory scoreToPickup = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
            new Translation2d(pickupXDistance * 0.25, avoidCollisionYOffset),
            new Translation2d(pickupXDistance * 0.9, 0)),
            new Pose2d(pickupXDistance, pickupYDistance, Rotation2d.fromDegrees(pickupRotation)),
            trajectoryConfig);

        // Trajectory with an inital pose
        // Trajectory pickupToScore = TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(pickupXDistance, pickupYDistance, new Rotation2d(pickupRotation)), 
        //     List.of(
        //     new Translation2d(pickupXDistance * 0.75, yOffset),
        //     new Translation2d(pickupXDistance * 0.25, yOffset)),
        //     new Pose2d(scoreXDistance, scoreYDistance, Rotation2d.fromDegrees(0)),
        //     trajectoryConfig);

        // Trajectory without an intial pose so we don't do unnecessary movement after vision pickup
        Trajectory pickupToScore = TrajectoryGenerator.generateTrajectory(
            List.of(
            new Pose2d(pickupXDistance * 0.75, avoidCollisionYOffset, Rotation2d.fromDegrees(pickupRotation * 0.75)),
            new Pose2d(pickupXDistance * 0.1, avoidCollisionYOffset, Rotation2d.fromDegrees(pickupRotation * 0.25)),
            new Pose2d(scoreXDistance, scoreYDistance, Rotation2d.fromDegrees(-10)))
            , trajectoryConfig);

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
        
        final StartPosition startPositionFinal = position;
        final Alliance allianceFinal = alliance;

        return race(
            sequence(
                
                // ======== Drop Preload Start ========
                Commands.parallel(
                    runOnce(() -> SmartDashboard.putString("Stage", "Start")),
                    runOnce(() -> swerveDrive.resetOdometry(scoreToPickup.getInitialPose())),
                    runOnce(() -> swerveDrive.stopModules())
                ),

                // Move arm and elevator, arm is moved 0.5 seconds after the elevator to prevent power chain from getting caught
                Commands.race(
                    Commands.waitSeconds(5), // Timeout
                    Commands.sequence(
                        Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScore)),
                        Commands.waitSeconds(0.5),

                        Commands.parallel( // End when target positions reached
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorScoreMid))
                        )
                    )
                ),
                waitSeconds(0.25),

                // Open claw/eject piece with rollers
                claw.setPower(0.5),

                // Wait to outtake
                Commands.waitSeconds(.5),

                // Close claw/stop rollers
                claw.setPower(0),

                // Parallel driving to pickup position and moving arm/elev to ready-to-pickup position
                Commands.deadline(
                    scoreToPickupCommand,

                    // Move arm to ready-to-pickup position
                    Commands.sequence(
                        Commands.deadline(
                            Commands.waitSeconds(2),
                            Commands.parallel( // End command once both arm and elevator have reached their target position
                                Commands.waitUntil(arm.atTargetPosition),
                                Commands.waitUntil(elevator.atTargetPosition),
                                Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                                Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow))
                            )
                        ),
                        Commands.deadline(
                            Commands.waitSeconds(2),
                            Commands.parallel( // End command once both arm and elevator have reached their target position
                                Commands.waitUntil(arm.atTargetPosition),
                                Commands.waitUntil(elevator.atTargetPosition),
                                Commands.runOnce(() -> arm.setTargetTicks(-328500 + 7950)),
                                Commands.runOnce(() -> elevator.setTargetTicks(-160000))
                            )
                        )
                    )
                ),
                runOnce(() -> swerveDrive.stopModules()),

                // Arm is moved to pick up cube, ends with arm/elev extended and cube in the claw
                vision.VisionPickupGroundNoArm(OBJECT_TYPE.CUBE),

                // Drive to score in parallel with arm moving to score position and elevator stowing
                Commands.parallel(
                    pickupToScoreCommand,

                    Commands.deadline(
                        Commands.waitSeconds(2),
                        Commands.parallel( // End command once both arm and elevator have reached their target position
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScore)),
                            Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorScoreMid))
                        )
                    )
                ),

                parallel(
                    runOnce(() -> swerveDrive.stopModules()),
                    runOnce(() -> SmartDashboard.putString("Stage", "Score 2"))
                ),

                // Drive to target and score, ends with arm/elev fully extended to score
                new TurnToAngle(0, swerveDrive),
                vision.VisionScoreNoArm(OBJECT_TYPE.CUBE, SCORE_POS.MID),
                // new TurnToAngle(0, swerveDrive),
                    
                // Stow the elevator, move arm to substation pos, and charge in parallel
                Commands.parallel(
                    Commands.deadline(
                        Commands.waitSeconds(2),
                        Commands.parallel( // End command once both arm and elevator have reached their target position
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                            Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow))
                        )
                    )
                )
            ),
            
            run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
            run(() -> elevator.moveMotionMagic(arm.getArmAngle()))
        );
    }

    public static CommandBase visionPreloadPickupChargeAuto(SwerveDrivetrain swerveDrive, VROOOOM vision, Arm arm, Elevator elevator, MotorClaw claw, StartPosition position, SCORE_POS scorePos, Alliance alliance) {
        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            SwerveAutoConstants.kMaxSpeedMetersPerSecond, SwerveAutoConstants.kMaxAccelerationMetersPerSecondSquared);

        double pickupXDistance = 0;
        double pickupYDistance = 0;
        double pickupRotation = 0;

        double avoidCollisionYOffset = 0; // So we don't bump into the charging station

        // for charging
        double yTranslation = 0;
        double yOvershoot = 0;

        if (alliance == Alliance.Red) {
            if (position == StartPosition.RIGHT) position = StartPosition.LEFT;
            if (position == StartPosition.LEFT) position = StartPosition.RIGHT;
        }

        switch (position) {
            // Reversed because gyro starts reversed
            case RIGHT:
                pickupXDistance = -3.73;
                pickupYDistance = 0.93;
                pickupRotation = -175;
                avoidCollisionYOffset = 0.25;
                yTranslation = 2.25;
                yOvershoot = 2.25;
                break;
            case LEFT:
                pickupXDistance = -3.73;
                pickupYDistance = -0.93;
                pickupRotation = 175; // Tested at Da Vinci, not accurate likely because of limelight placement
                avoidCollisionYOffset = -0.25; // Was 0.75 when tested at Da Vinci with only 1 waypoint, but now we're using 2
                yTranslation = -2.25;
                yOvershoot = -2.25;
                break;
            case MIDDLE:
                pickupXDistance = -5; // TODO: Measure IRL
                pickupYDistance = -0.3;
                pickupRotation = -165;
                break;
        }

        // Negate all measurements related to the y-axis because we are on the opposite side of the field
        if (alliance == Alliance.Red) {
            if (position != StartPosition.MIDDLE) {
                pickupYDistance *= -1;
                avoidCollisionYOffset *= -1;
                pickupRotation *= -1;
            }
        }

        Trajectory scoreToPickup = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
            new Translation2d(pickupXDistance * 0.25, avoidCollisionYOffset),
            new Translation2d(pickupXDistance * 0.9, 0)),
            new Pose2d(pickupXDistance, pickupYDistance, Rotation2d.fromDegrees(pickupRotation)),
            trajectoryConfig);
        
        Trajectory chargeTrajectory = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(-4.0, yOvershoot * 0.75, Rotation2d.fromDegrees(175)),
                new Pose2d(-3.5, yOvershoot, Rotation2d.fromDegrees(179.9)),
                new Pose2d(-1.3, yTranslation, Rotation2d.fromDegrees(179.9))),
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
        
        SwerveControllerCommand chargeCommand = new SwerveControllerCommand(
            chargeTrajectory, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return race(
            sequence(
                // ======== Drop Preload Start ========

                parallel(
                    runOnce(() -> SmartDashboard.putString("Stage", "Start")),
                    runOnce(() -> swerveDrive.resetOdometry(scoreToPickup.getInitialPose())),
                    runOnce(() -> swerveDrive.stopModules())
                ),

                // Move arm and elevator, arm is moved 0.5 seconds after the elevator to prevent power chain from getting caught
                Commands.race(
                    Commands.waitSeconds(5), // Timeout
                    Commands.sequence(
                        Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScore)),
                        Commands.waitSeconds(0.5),

                        Commands.parallel( // End when target positions reached
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorScoreHigh))
                        )
                    )
                ),

                // Open claw/eject piece with rollers
                claw.setPower(0.5),

                // Wait to outtake
                Commands.waitSeconds(.5),

                // Close claw/stop rollers
                claw.setPower(0),

                // Parallel driving to pickup position and moving arm/elev to ready-to-pickup position
                Commands.deadline(
                    scoreToPickupCommand,

                    // Move arm to ready-to-pickup position
                    Commands.sequence(
                        Commands.deadline(
                            Commands.waitSeconds(2),
                            Commands.parallel( // End command once both arm and elevator have reached their target position
                                Commands.waitUntil(arm.atTargetPosition),
                                Commands.waitUntil(elevator.atTargetPosition),
                                Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                                Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow))
                            )
                        ),
                        Commands.waitSeconds(1),
                        Commands.deadline(
                            Commands.waitSeconds(2),
                            Commands.parallel( // End command once both arm and elevator have reached their target position
                                Commands.waitUntil(arm.atTargetPosition),
                                Commands.waitUntil(elevator.atTargetPosition),
                                Commands.runOnce(() -> arm.setTargetTicks(-328500)),
                                Commands.runOnce(() -> elevator.setTargetTicks(-160000))
                            )
                        )
                    )
                ),
                runOnce(() -> swerveDrive.stopModules()),

                // Arm is moved to pick up cube, ends with arm/elev extended and cube in the claw
                vision.VisionPickupGroundNoArm(OBJECT_TYPE.CUBE),

                // Drive to score in parallel with arm moving to score position and elevator stowing
                Commands.parallel(
                    chargeCommand,
                    Commands.deadline(
                        Commands.waitSeconds(2),
                        Commands.parallel( // End command once both arm and elevator have reached their target position
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                            Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow))
                        )
                    )
                ),

                runOnce(() -> swerveDrive.stopModules()),
                runOnce(() -> SmartDashboard.putString("Stage", "Charging")),
                new TheGreatBalancingAct(swerveDrive)
            ),
            run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
            run(() -> elevator.moveMotionMagic(arm.getArmAngle()))
        );
    }

    /**
     * Finds April Tag ID when command is executed
     * 
     * @return Command that displays April Tag ID on SmartDashboard
     */
    public static CommandBase debugVisionAprilTagAuto(VROOOOM vision) {
        final int aprilTagID = vision.getAprilTagID();
        return Commands.run(() -> SmartDashboard.putNumber("April Tag ID Auto", aprilTagID));
    }

    /**
     * Starting position: Facing any cube shelf (with bumpers flush to grid) with a cube preload.
     * 
     * Auto sequence:
     * 1. Scores preload
     * 2. Goes to pickup cone with vision assist
     * 3. Goes to score cone with vision assist
     * 4. Charges with gyro assist
     * 
     * Field placements:
     * - If starting on the left, the leftmost game piece should be a cone
     * - If starting on the right, the rightmost game piece should be a cone
     * - If starting in the middle, the second game piece from the right should be a cone
     * 
     * @return Auto command
     */
    public static CommandBase visionAutoChoose(SwerveDrivetrain swerveDrive, VROOOOM vision, Arm arm, Elevator elevator, 
            MotorClaw claw, Alliance selectedAlliance, StartPosition selectedStartPosition) {
        final int aprilTagID = vision.getAprilTagID();

        // Default values
        Alliance alliance = Alliance.Red;
        StartPosition startPosition = StartPosition.LEFT;

        switch (aprilTagID) {
            // Red alliance positions
            case 1:
                alliance = Alliance.Red;
                startPosition = StartPosition.LEFT;
                break;

            case 2:
                alliance = Alliance.Red;
                startPosition = StartPosition.MIDDLE;
                break;

            case 3: 
                alliance = Alliance.Red;
                startPosition = StartPosition.RIGHT;
                break;

            // Blue alliance positions
            case 6:
                alliance = Alliance.Blue;
                startPosition = StartPosition.LEFT;
                break;

            case 7: 
                alliance = Alliance.Blue;
                startPosition = StartPosition.MIDDLE;
                break;

            case 8:
                alliance = Alliance.Blue;
                startPosition = StartPosition.RIGHT;
                break;

            default: // Case that runs if no april tag is found, in which april tag ID will be -1
                alliance = selectedAlliance;
                startPosition = selectedStartPosition;
                break;
        }

        final Alliance allianceFinal = alliance;
        final StartPosition startPositionFinal = startPosition;

        return visionPreloadPickupScoreCharge(swerveDrive, vision, arm, elevator, claw, allianceFinal, startPositionFinal, SCORE_POS.HIGH);
    }

    /**
     * Start with the swerve drive facing the driver at either the rightmost cone grid, the leftmost cone grid, or directly in front of the charging station (middle)
     * @param swerveDrive 
     * @return Command to reset odometry run auto to go onto charging station then run balancing auto
     */
    public static CommandBase visionChargeAuto(SwerveDrivetrain swerveDrive, StartPosition startPos, Alliance alliance, double waitTime, boolean goAround) {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
        
        double yTranslation = 0;
        double yOvershoot = 0;

        switch (startPos) {
            case LEFT:
                yTranslation = -2.4;
                yOvershoot = -2.4;
                break;
            case RIGHT:
                yTranslation = 2.4;
                yOvershoot = 2.4;
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
                List.of(
                    new Pose2d(-0.5, yOvershoot * 0.2, Rotation2d.fromDegrees(0)),
                    new Pose2d(-0.5, yOvershoot, Rotation2d.fromDegrees(0)),
                    new Pose2d(-2.3, yTranslation - 0.01, Rotation2d.fromDegrees(0))
                ), // -2
                trajectoryConfig);
        } else {
            trajectory = TrajectoryGenerator.generateTrajectory(
                List.of(
                    new Pose2d(0, 0, new Rotation2d(0)), 
                    new Pose2d(-3.5, yTranslation / 4, new Rotation2d(0)), 
                    new Pose2d(-3.5, yTranslation + 0.01, new Rotation2d(0)), 
                    new Pose2d(-2, yTranslation - 0.01, Rotation2d.fromDegrees(0))
                ), 
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
            waitSeconds(waitTime),
            autoCommand,
            new TheGreatBalancingAct(swerveDrive)
            // new TowSwerve(swerveDrive)
        );
    }
}
