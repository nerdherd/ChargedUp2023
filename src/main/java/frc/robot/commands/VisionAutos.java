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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
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
                            Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorScoreHigh))
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
                    Commands.waitSeconds(2),
                    Commands.parallel( // End command once both arm and elevator have reached their target position
                        Commands.waitUntil(arm.atTargetPosition),
                        Commands.waitUntil(elevator.atTargetPosition),
                        Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                        Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow))
                    )
                ),

                // Commands.deadline(
                scoreToPickupCommand,

                    // Move arm to ready-to-pickup position
                    // Commands.sequence(
                        // Commands.deadline(
                        //     Commands.waitSeconds(2),
                        //     Commands.parallel( // End command once both arm and elevator have reached their target position
                        //         Commands.waitUntil(arm.atTargetPosition),
                        //         Commands.waitUntil(elevator.atTargetPosition),
                        //         Commands.runOnce(() -> arm.setTargetTicks(-328500 + 7950)),
                        //         Commands.runOnce(() -> elevator.setTargetTicks(-160000))
                        //     )
                        // )
                    // )
                // ),
                runOnce(() -> swerveDrive.stopModules()),

                // Arm is moved to pick up cube, ends with arm/elev extended and cube in the claw
                vision.VisionPickupGroundNoArm(OBJECT_TYPE.CUBE, claw), // Added claw parameter to get rid of error

                Commands.deadline(
                    Commands.waitSeconds(2),
                    Commands.parallel( // End command once both arm and elevator have reached their target position
                        Commands.waitUntil(arm.atTargetPosition),
                        Commands.waitUntil(elevator.atTargetPosition),
                        Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                        Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow))
                    )
                ),

                // Drive to score in parallel with arm moving to score position and elevator stowing
                // Commands.parallel(
                    pickupToScoreCommand,
                // ),

                parallel(
                    runOnce(() -> swerveDrive.stopModules()),
                    runOnce(() -> SmartDashboard.putString("Stage", "Score 2"))
                ),

                // Drive to target and score, ends with arm/elev fully extended to score
                new TurnToAngle(0, swerveDrive),

                Commands.deadline(
                    Commands.waitSeconds(2),
                    Commands.parallel( // End command once both arm and elevator have reached their target position
                        Commands.waitUntil(arm.atTargetPosition),
                        Commands.waitUntil(elevator.atTargetPosition),
                        Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScore)),
                        Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorScoreMid))
                    )
                ),

                

                
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
        ).finallyDo((x) -> swerveDrive.getImu().setOffset(180));
    }
    
    public static CommandBase zoomTwoPieceAuto(SwerveDrivetrain swerveDrive, VROOOOM vision, Arm arm, Elevator elevator, MotorClaw claw, 
    Alliance alliance){
        PIDController trajectoryXController = new PIDController(kPXController, kIXController, kDXController);
        PIDController trajectoryYController = new PIDController(kPYController, kIYController, kDYController);
        ProfiledPIDController trajectoryThetaController = new ProfiledPIDController(kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);

        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            SwerveAutoConstants.kTwoPieceSpeedMetersPerSecond, SwerveAutoConstants.kTwoPieceAccelerationMetersPerSecondSquared);

        double allianceFactor = 1.0;
        if (alliance == Alliance.Red) {
            allianceFactor = -1.0;
        }
        
        //trajectory stuff

        Trajectory zoooomToCube = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                new Translation2d(0.2, 0.2 * allianceFactor)
                //new Translation2d(-1.8, -0.4)
            ),
            new Pose2d(3.9, 0.2 * allianceFactor, Rotation2d.fromDegrees(0)),
            trajectoryConfig);

        Trajectory cubeToZoooom = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(3.9, 0.2 * allianceFactor, Rotation2d.fromDegrees(180)),
                new Pose2d(0.8, 0.2 * allianceFactor, Rotation2d.fromDegrees(180)),
                // new Pose2d(-0.8, -1.0, Rotation2d.fromDegrees(0)),
                new Pose2d(0.2, 1.0 * allianceFactor, Rotation2d.fromDegrees(180))
            ),
            trajectoryConfig);

        Trajectory zoooomPartTwo = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.2, 1.0 * allianceFactor, Rotation2d.fromDegrees(179.9)),
                new Pose2d(0.8, 0.2 * allianceFactor, Rotation2d.fromDegrees(179.9)),
                new Pose2d(3.8, 0.2 * allianceFactor, Rotation2d.fromDegrees(179.9)),
                new Pose2d(4, 2.2 * allianceFactor, Rotation2d.fromDegrees(179.9))
            ),
            trajectoryConfig);

        SwerveControllerCommand zoooomToCubeCommand = new SwerveControllerCommand(
            zoooomToCube, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            trajectoryXController, trajectoryYController, trajectoryThetaController, swerveDrive::setModuleStates, swerveDrive);

        SwerveControllerCommand cubeToZoooomCommand = new SwerveControllerCommand(
            cubeToZoooom, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            trajectoryXController, trajectoryYController, trajectoryThetaController, swerveDrive::setModuleStates, swerveDrive);

        SwerveControllerCommand zoooomPartTwoCommand = new SwerveControllerCommand(
            zoooomPartTwo, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            trajectoryXController, trajectoryYController, trajectoryThetaController, swerveDrive::setModuleStates, swerveDrive);

        final int armPosFinal = ArmConstants.kArmScore;
        final int elevatorPosFinal = ElevatorConstants.kElevatorScoreHigh;

        return Commands.race(
            //init
            Commands.sequence(
                parallel(
                    runOnce(() -> SmartDashboard.putString("Stage", "Start")),
                    runOnce(() -> swerveDrive.resetOdometry(zoooomToCube.getInitialPose())),
                    runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                    runOnce(() -> swerveDrive.stopModules())
                ),

                //trajectory to cube
                Commands.runOnce(() -> SmartDashboard.putString("Moved On", "zoomin")),
                Commands.parallel(
                    claw.setPower(ClawConstants.kIntakePower),
                    zoooomToCubeCommand,
                    Commands.deadline(
                        Commands.waitSeconds(2),
                        Commands.runOnce(() -> SmartDashboard.putString("Stage", "Score")),
                        Commands.sequence(
                            Commands.runOnce(() -> arm.setTargetTicks((ArmConstants.kArmScore + ArmConstants.kArmGroundPickupVision) /2)),
                            Commands.waitSeconds(0.5),
                            Commands.waitUntil(arm.atTargetPosition)
                        )
                    )
                ),
                runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                runOnce(() -> swerveDrive.stopModules()),

                // // new TurnToAngle(170, swerveDrive),
                // runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                // runOnce(() -> swerveDrive.stopModules()),

                vision.VisionPickupGroundNoArm(OBJECT_TYPE.CUBE, claw),

                runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                runOnce(() -> swerveDrive.stopModules()),

                // Cube should be in the claw at this point

                Commands.parallel(
                    new TurnToAngle(180, swerveDrive),
                    claw.setPower(ClawConstants.kIntakeNeutralPower),

                    // Drop arm and elevator so the game piece can be intook
                    Commands.deadline(
                        Commands.waitSeconds(2),
                        Commands.runOnce(() -> SmartDashboard.putString("Stage", "Score")),
                        Commands.sequence(
                            Commands.runOnce(() -> arm.setTargetTicks((ArmConstants.kArmScore + ArmConstants.kArmGroundPickupVision) / 2)), // Halfway between scoring and ground pickup
                            Commands.waitSeconds(0.5),
                            Commands.waitUntil(arm.atTargetPosition)
                        )
                    )
                ),
                cubeToZoooomCommand,
                runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                runOnce(() -> swerveDrive.stopModules()),

                claw.setPower(ClawConstants.kOuttakePower),
                Commands.waitSeconds(0.5),
                claw.setPowerZero(),
                
                zoooomPartTwoCommand,
                claw.setPower(ClawConstants.kIntakePower),

                runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                runOnce(() -> swerveDrive.stopModules()),

                new TurnToAngle(0, swerveDrive),
                Commands.parallel(
                    Commands.deadline(
                        Commands.waitSeconds(2),
                        Commands.runOnce(() -> SmartDashboard.putString("Stage", "Score")),
                        Commands.sequence(
                            Commands.runOnce(() -> arm.setTargetTicks((ArmConstants.kArmScore + ArmConstants.kArmGroundPickupVision) /2)),
                            Commands.waitSeconds(0.5),
                            Commands.waitUntil(arm.atTargetPosition)
                        )
                    )
                ),
                // //vision pickup
                // // Arm is moved to pick up cube, ends with arm/elev extended and cube in the claw
                vision.VisionPickupGroundNoArm(OBJECT_TYPE.CUBE, claw),
                claw.setPower(ClawConstants.kIntakeNeutralPower)
            ),
            run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
            run(() -> elevator.moveMotionMagic(arm.getArmAngle()))
        );
    }

    public static CommandBase cableZoomTwoPieceAuto(SwerveDrivetrain swerveDrive, VROOOOM vision, Arm arm, Elevator elevator, MotorClaw claw, 
    Alliance alliance){
        PIDController trajectoryXController = new PIDController(kPXController, kIXController, kDXController);
        PIDController trajectoryYController = new PIDController(kPYController, kIYController, kDYController);
        ProfiledPIDController trajectoryThetaController = new ProfiledPIDController(kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);

        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            SwerveAutoConstants.kTwoPieceSpeedMetersPerSecond, SwerveAutoConstants.kTwoPieceAccelerationMetersPerSecondSquared);

        double allianceFactor = -1.0;
        if (alliance == Alliance.Red) {
            allianceFactor = 1.0;
        }
        
        //trajectory stuff

        Trajectory zoooomToCube = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                new Translation2d(0.2, 0.2 * allianceFactor)
                //new Translation2d(-1.8, -0.4)
            ),
            new Pose2d(3.9, 0.2 * allianceFactor, Rotation2d.fromDegrees(0)),
            trajectoryConfig);

        Trajectory cubeToZoooom = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(3.9, 0.2 * allianceFactor, Rotation2d.fromDegrees(180)),
                new Pose2d(0.8, 0.2 * allianceFactor, Rotation2d.fromDegrees(180)),
                // new Pose2d(-0.8, -1.0, Rotation2d.fromDegrees(0)),
                new Pose2d(0.2, 1.0 * allianceFactor, Rotation2d.fromDegrees(180))
            ),
            trajectoryConfig);

        Trajectory zoooomPartTwo = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.2, 1.0 * allianceFactor, Rotation2d.fromDegrees(179.9)),
                new Pose2d(0.8, 0.2 * allianceFactor, Rotation2d.fromDegrees(179.9)),
                new Pose2d(3.9, 0.2 * allianceFactor, Rotation2d.fromDegrees(179.9)),
                new Pose2d(4.2, 2.2 * allianceFactor, Rotation2d.fromDegrees(179.9))
            ),
            trajectoryConfig);

        SwerveControllerCommand zoooomToCubeCommand = new SwerveControllerCommand(
            zoooomToCube, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            trajectoryXController, trajectoryYController, trajectoryThetaController, swerveDrive::setModuleStates, swerveDrive);

        SwerveControllerCommand cubeToZoooomCommand = new SwerveControllerCommand(
            cubeToZoooom, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            trajectoryXController, trajectoryYController, trajectoryThetaController, swerveDrive::setModuleStates, swerveDrive);

        SwerveControllerCommand zoooomPartTwoCommand = new SwerveControllerCommand(
            zoooomPartTwo, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            trajectoryXController, trajectoryYController, trajectoryThetaController, swerveDrive::setModuleStates, swerveDrive);

        final int armPosFinal = ArmConstants.kArmScore;
        final int elevatorPosFinal = ElevatorConstants.kElevatorScoreHigh;

        return Commands.race(
            //init
            Commands.sequence(
                parallel(
                    runOnce(() -> SmartDashboard.putString("Stage", "Start")),
                    runOnce(() -> swerveDrive.resetOdometry(zoooomToCube.getInitialPose())),
                    runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                    runOnce(() -> swerveDrive.stopModules())
                ),

                //trajectory to cube
                Commands.runOnce(() -> SmartDashboard.putString("Moved On", "zoomin")),
                Commands.parallel(
                    claw.setPower(ClawConstants.kIntakePower),
                    zoooomToCubeCommand,
                    Commands.deadline(
                        Commands.waitSeconds(2),
                        Commands.runOnce(() -> SmartDashboard.putString("Stage", "Score")),
                        Commands.sequence(
                            Commands.waitSeconds(1),
                            Commands.runOnce(() -> arm.setTargetTicks((ArmConstants.kArmScore + ArmConstants.kArmGroundPickupVision) /2)),
                            Commands.waitSeconds(0.5),
                            Commands.waitUntil(arm.atTargetPosition)
                        )
                    )
                ),
                runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                runOnce(() -> swerveDrive.stopModules()),

                // // new TurnToAngle(170, swerveDrive),
                // runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                // runOnce(() -> swerveDrive.stopModules()),

                vision.VisionPickupGroundNoArm(OBJECT_TYPE.CUBE, claw),

                runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                runOnce(() -> swerveDrive.stopModules()),

                // Cube should be in the claw at this point

                Commands.parallel(
                    new TurnToAngle(180, swerveDrive),
                    claw.setPower(ClawConstants.kIntakeNeutralPower),

                    // Drop arm and elevator so the game piece can be intook
                    Commands.deadline(
                        Commands.waitSeconds(2),
                        Commands.runOnce(() -> SmartDashboard.putString("Stage", "Score")),
                        Commands.sequence(
                            Commands.runOnce(() -> arm.setTargetTicks((ArmConstants.kArmScore + ArmConstants.kArmGroundPickupVision) / 2)), // Halfway between scoring and ground pickup
                            Commands.waitSeconds(0.5),
                            Commands.waitUntil(arm.atTargetPosition)
                        )
                    )
                ),
                cubeToZoooomCommand,
                runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                runOnce(() -> swerveDrive.stopModules()),

                claw.setPower(ClawConstants.kOuttakePower),
                Commands.waitSeconds(0.5),
                claw.setPowerZero(),
                
                zoooomPartTwoCommand,
                claw.setPower(ClawConstants.kIntakePower),

                runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                runOnce(() -> swerveDrive.stopModules()),

                new TurnToAngle(0, swerveDrive),
                Commands.parallel(
                    Commands.deadline(
                        Commands.waitSeconds(2),
                        Commands.runOnce(() -> SmartDashboard.putString("Stage", "Score")),
                        Commands.sequence(
                            Commands.runOnce(() -> arm.setTargetTicks((ArmConstants.kArmScore + ArmConstants.kArmGroundPickupVision) /2)),
                            Commands.waitSeconds(0.5),
                            Commands.waitUntil(arm.atTargetPosition)
                        )
                    )
                ),
                // //vision pickup
                // // Arm is moved to pick up cube, ends with arm/elev extended and cube in the claw
                vision.VisionPickupGroundNoArm(OBJECT_TYPE.CUBE, claw),
                claw.setPower(ClawConstants.kIntakeNeutralPower)
            ),
            run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
            run(() -> elevator.moveMotionMagic(arm.getArmAngle()))
        );
    }
}