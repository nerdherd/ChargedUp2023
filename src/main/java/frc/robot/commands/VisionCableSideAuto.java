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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

public class VisionCableSideAuto {
    
    public static CommandBase LowAuto(SwerveDrivetrain swerveDrive, VROOOOM vision, Arm arm, Elevator elevator, MotorClaw claw, 
    Alliance alliance){
        PIDController trajectoryXController = new PIDController(kPXController, kIXController, kDXController);
        PIDController trajectoryYController = new PIDController(kPYController, kIYController, kDYController);
        ProfiledPIDController trajectoryThetaController = 
            new ProfiledPIDController(kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);

        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            SwerveAutoConstants.kMaxSpeedMetersPerSecond * 0.75, SwerveAutoConstants.kMaxAccelerationMetersPerSecondSquared);

        double zoooomAllianceThingy = 1.0;
        int atagId = 8;
        if (alliance == Alliance.Red) {
            zoooomAllianceThingy = -1.0;
            atagId = 1;
        }
        
        //trajectory stuff

        /*Trajectory zoooomToCube_A = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                new Translation2d(0.18, -0.18 * zoooomAllianceThingy) 
            ),
            new Pose2d(2.4, -0.18 * zoooomAllianceThingy, Rotation2d.fromDegrees(0)),
            trajectoryConfig);

        Trajectory zoooomToCube_B = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(2.5, -0.18 * zoooomAllianceThingy, Rotation2d.fromDegrees(0)),
                new Pose2d(4.4, -0.18 * zoooomAllianceThingy, Rotation2d.fromDegrees(0))
            ),
            trajectoryConfig);*/

        Trajectory zoooomToCube = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                new Translation2d(0.2, 0 * zoooomAllianceThingy),
                new Translation2d(-0.2, 0 * zoooomAllianceThingy), // push the cube to hybrid zone
                //new Translation2d(0, 0 * zoooomAllianceThingy), // comment it out, grid holds previous -0.2
                new Translation2d(0.2, -0.2 * zoooomAllianceThingy), 
                new Translation2d(2.5, -0.2 * zoooomAllianceThingy),
                new Translation2d(4.0, -0.2 * zoooomAllianceThingy)
            ),
            new Pose2d(4.4, -0.2 * zoooomAllianceThingy, Rotation2d.fromDegrees(0)),
            trajectoryConfig);

        /*Trajectory cubeToZoooom_A = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(3.6, -0.1 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)),
                new Pose2d(2.0, -0.1 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9))
            ),
            trajectoryConfig);

        Trajectory cubeToZoooom_B = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(1.9, -0.1 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)), 
                new Pose2d(1.5, -0.1 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)),
                new Pose2d(0, -0.1 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)),
                new Pose2d(0, -0.61 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)),
                new Pose2d(-0.5, -0.61 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9))
            ),
            trajectoryConfig);*/

        Trajectory cubeToZoooom = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(3.5, -0.1 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)),
                new Pose2d(2.0, -0.1 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)),
                new Pose2d(1.5, -0.1 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)),
                new Pose2d(-0.2, -0.1 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)),
                new Pose2d(-0.2, -0.62 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)),
                new Pose2d(-0.7, -0.62 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9))
            ),
            trajectoryConfig);
        

        SwerveControllerCommand zoooomToCubeCommand = new SwerveControllerCommand(
            zoooomToCube, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            trajectoryXController, trajectoryYController, trajectoryThetaController, swerveDrive::setModuleStates, swerveDrive);

        /*SwerveControllerCommand zoooomToCubeCommand_B = new SwerveControllerCommand(
            zoooomToCube_B, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            trajectoryXController, trajectoryYController, trajectoryThetaController, swerveDrive::setModuleStates, swerveDrive);*/

        SwerveControllerCommand cubeToZoooomCommand = new SwerveControllerCommand(
            cubeToZoooom, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            trajectoryXController, trajectoryYController, trajectoryThetaController, swerveDrive::setModuleStates, swerveDrive);

        /*SwerveControllerCommand cubeToZoooomCommand_B = new SwerveControllerCommand(
            cubeToZoooom_B, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            trajectoryXController, trajectoryYController, trajectoryThetaController, swerveDrive::setModuleStates, swerveDrive);*/

        final int atagIdFinal = atagId;

        return Commands.race(
            Commands.waitSeconds(15), // TODO DEL
            //init
            Commands.sequence(

                Commands.runOnce(() -> swerveDrive.resetOdometry(zoooomToCube.getInitialPose())),

                //trajectory to cube
                Commands.parallel(
                    zoooomToCubeCommand,
                    Commands.sequence(
                        Commands.waitSeconds(1.5),
                        runOnce(() -> arm.setTargetTicks((ArmConstants.kArmStow) )) // to be safe
                    ),
                    Commands.runOnce(() -> vision.initVisionPickupOnGround(OBJECT_TYPE.CUBE))
                ),

                Commands.race(
                    new RunCommand(() -> vision.driveToCubeOnGround(claw, 5), arm, elevator, claw, swerveDrive).until(vision.cameraStatusSupplier),
                    Commands.waitSeconds(20) // kill this auto
                    // TODO need add protection here!!!!!!
                ),

                Commands.parallel(
                    claw.setPower(-0.36),
                    Commands.deadline(
                        Commands.waitSeconds(1.2),
                        sequence(
                            runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup)),
                            waitUntil(arm.atTargetPosition)
                        )
                    )
                ),

                Commands.waitSeconds(0.1),

                Commands.parallel(
                    runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                    Commands.sequence(
                        waitSeconds(0.2),
                        new TurnToAngle(-179.9, swerveDrive)
                    )
                ),
                // Close claw/stop claw intake rollers/low background rolling to keep control of game piece
                claw.setPower(-0.20),

                Commands.parallel(
                    cubeToZoooomCommand,
                    Commands.runOnce(() -> vision.initVisionPickupOnGround(OBJECT_TYPE.ATAG))
                ),

                new TurnToAngle(-179.9, swerveDrive),
                
                parallel (
                    Commands.race(
                        new RunCommand(() -> vision.driveToGridTag(claw, atagIdFinal), arm, elevator, claw, swerveDrive).until(vision.cameraStatusSupplier),
                        Commands.waitSeconds(3) 
                    )/* ,

                    //Drop arm High drop off
                    deadline(
                        waitSeconds(2),
                        sequence(
                            runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScoreCubeHigh)),
                            waitSeconds(0.5),
                            waitUntil(arm.atTargetPosition)
                        )
                    )*/
                ),

                /*Commands.deadline(
                    Commands.waitSeconds(0.5),
                    sequence(
                        runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorScoreHighCube)),
                        waitSeconds(0.5),
                        waitUntil(elevator.atTargetPosition)
                    )
                ),*/

                claw.setPower(0.3)
                
            ),

            run(() -> arm.moveArmMotionMagic(elevator.percentExtended()))
            // run(() -> elevator.moveMotionMagic(arm.getArmAngle()))
        );
    }















    public static CommandBase HighAuto(SwerveDrivetrain swerveDrive, VROOOOM vision, Arm arm, Elevator elevator, MotorClaw claw, 
    Alliance alliance){
        PIDController trajectoryXController = new PIDController(kPXController, kIXController, kDXController);
        PIDController trajectoryYController = new PIDController(kPYController, kIYController, kDYController);
        ProfiledPIDController trajectoryThetaController = 
            new ProfiledPIDController(kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);

        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            SwerveAutoConstants.kMaxSpeedMetersPerSecond * 0.75, SwerveAutoConstants.kMaxAccelerationMetersPerSecondSquared);

        double zoooomAllianceThingy = 1.0;
        int atagId = 8;
        if (alliance == Alliance.Red) {
            zoooomAllianceThingy = -1.0;
            atagId = 1;
        }
        
        //trajectory stuff

        Trajectory zoooomToCube = TrajectoryGenerator.generateTrajectory(
            new Pose2d(-0.125, 0, new Rotation2d(0)), 
            List.of(
                new Translation2d(-0.3, -0.2 * zoooomAllianceThingy), 
                new Translation2d(-2.5, -0.2 * zoooomAllianceThingy), 
                new Translation2d(-4.3, -0.2 * zoooomAllianceThingy)), 
            new Pose2d(-4.5, -0.2 * zoooomAllianceThingy, Rotation2d.fromDegrees(0)),
            trajectoryConfig);

        Trajectory cubeToZoooom = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(-4.4, -0.1 * zoooomAllianceThingy, Rotation2d.fromDegrees(0)),
                new Pose2d(-2.0, -0.1 * zoooomAllianceThingy, Rotation2d.fromDegrees(0)),
                new Pose2d(-1.5, -0.1 * zoooomAllianceThingy, Rotation2d.fromDegrees(0)),
                new Pose2d(0, -0.1 * zoooomAllianceThingy, Rotation2d.fromDegrees(0)),
                new Pose2d(0, -0.61 * zoooomAllianceThingy, Rotation2d.fromDegrees(0)),
                new Pose2d(-0.5, -0.61 * zoooomAllianceThingy, Rotation2d.fromDegrees(0))
            ),
            trajectoryConfig);
        

        SwerveControllerCommand zoooomToCubeCommand = new SwerveControllerCommand(
            zoooomToCube, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            trajectoryXController, trajectoryYController, trajectoryThetaController, swerveDrive::setModuleStates, swerveDrive);

        SwerveControllerCommand cubeToZoooomCommand = new SwerveControllerCommand(
            cubeToZoooom, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            trajectoryXController, trajectoryYController, trajectoryThetaController, swerveDrive::setModuleStates, swerveDrive);

        final int atagIdFinal = atagId;

        return Commands.race(
            Commands.waitSeconds(15), // TODO DEL
            //init
            Commands.sequence(

                Commands.race(
                    waitSeconds(5),
                    Commands.sequence(
                        parallel(
                            sequence(
                                claw.setPower(-0.35),
                                waitSeconds(0.25),
                                claw.setPower(-0.25)
                            ),
                            deadline(
                                waitSeconds(2),
                                runOnce(() -> SmartDashboard.putString("Stage", "Score")),
                                sequence(
                                    runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScore)),
                                    waitSeconds(0.5),
                                    waitUntil(arm.atTargetPosition)
                                ),
                                sequence(
                                    waitSeconds(0.5),
                                    runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorScoreHigh)),
                                    waitSeconds(0.5),
                                    waitUntil(elevator.atTargetPosition)
                                )
                            )
                        ),

                        waitSeconds(0.25),
                        claw.setPower(0.3),
                        waitSeconds(0.5),
                        claw.setPowerZero(),
                
                        deadline(
                            waitSeconds(0.5),
                            runOnce(() -> SmartDashboard.putString("Stage", "Stow")),
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
                        )
                    )
                ),

                //TODO confirm if we need it!!!!!!
                //Commands.runOnce(() -> swerveDrive.resetOdometry(zoooomToCube.getInitialPose()))

                //trajectory to cube
                Commands.parallel(
                    zoooomToCubeCommand,
                    Commands.sequence(
                        Commands.waitSeconds(1.5),
                        runOnce(() -> arm.setTargetTicks((ArmConstants.kArmScoreCubeHigh) )) // to be safe
                    ),
                    Commands.runOnce(() -> vision.initVisionPickupOnGround(OBJECT_TYPE.CUBE))
                ),
                new TurnToAngle(179.9, swerveDrive),

                Commands.race(
                    new RunCommand(() -> vision.driveToCubeOnGround(claw, 5), arm, elevator, claw, swerveDrive).until(vision.cameraStatusSupplier),
                    Commands.waitSeconds(20) // kill this auto
                    // TODO need add protection here!!!!!!
                ),

                Commands.parallel(
                    claw.setPower(-0.36),
                    Commands.deadline(
                        Commands.waitSeconds(1.2),
                        sequence(
                            runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup)),
                            waitUntil(arm.atTargetPosition)
                        )
                    )
                ),

                Commands.waitSeconds(0.1),

                Commands.parallel(
                    runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                    Commands.sequence(
                        waitSeconds(0.2),
                        new TurnToAngle(0, swerveDrive)
                    )
                ),
                // Close claw/stop claw intake rollers/low background rolling to keep control of game piece
                claw.setPower(-0.20),

                Commands.parallel(
                    cubeToZoooomCommand,
                    Commands.runOnce(() -> vision.initVisionPickupOnGround(OBJECT_TYPE.ATAG))
                ),

                new TurnToAngle(0, swerveDrive),

                parallel (
                    Commands.race(
                        new RunCommand(() -> vision.driveToGridTag(claw, atagIdFinal), arm, elevator, claw, swerveDrive).until(vision.cameraStatusSupplier),
                        Commands.waitSeconds(3) 
                    )/* ,

                    //Drop arm high drop off
                    Commands.deadline( // TODO: Fix this and the other two Deadline arm Commands
                        Commands.waitSeconds(0.5),
                        sequence(
                            runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScoreCubeMid)),
                            waitSeconds(0.5),
                            waitUntil(arm.atTargetPosition)
                        )
                    )*/
                ),

                claw.setPower(0.3)
                
            ),
            
            run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
            run(() -> elevator.moveMotionMagic(arm.getArmAngle()))
        );
    }
}
