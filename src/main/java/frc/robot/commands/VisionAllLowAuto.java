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

public class VisionAllLowAuto {
    
    public static CommandBase ThreeCubesAuto(SwerveDrivetrain swerveDrive, VROOOOM vision, Arm arm, Elevator elevator, MotorClaw claw, 
    Alliance alliance){
        PIDController trajectoryXController = new PIDController(kPXController, kIXController, kDXController);
        PIDController trajectoryYController = new PIDController(kPYController, kIYController, kDYController);
        ProfiledPIDController trajectoryThetaController = 
            new ProfiledPIDController(kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);

        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            SwerveAutoConstants.kMaxSpeedMetersPerSecond, SwerveAutoConstants.kMaxAccelerationMetersPerSecondSquared);

        double zoooomAllianceThingy = 1.0;;
        if (alliance == Alliance.Red) {
            zoooomAllianceThingy = -1.0;
        }
        
        //trajectory stuff

        Trajectory zoooomToCube = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                new Translation2d(0.2, 0.2) // only for blue side right now
                //new Translation2d(-1.8, -0.4)
            ),
            new Pose2d(3.6, 0.2, Rotation2d.fromDegrees(0)),
            trajectoryConfig);

        Trajectory cubeToZoooom = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(3.6, 0.2, Rotation2d.fromDegrees(180)),
                new Pose2d(0.8, 0.2, Rotation2d.fromDegrees(180)),
                // new Pose2d(-0.8, -1.0, Rotation2d.fromDegrees(0)),
                new Pose2d(0.2, 1.0, Rotation2d.fromDegrees(180))
            ),
            trajectoryConfig);

        Trajectory zoooomPartTwo = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0.2, 1.0, Rotation2d.fromDegrees(179.9)),
                new Pose2d(0.8, 0.2, Rotation2d.fromDegrees(179.9)),
                new Pose2d(3.6, 0.2, Rotation2d.fromDegrees(179.9)),
                new Pose2d(3.6, 2.2, Rotation2d.fromDegrees(179.9))
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
            Commands.waitSeconds(15), // TODO DEL
            //init
            Commands.sequence(
                parallel(
                    //runOnce(() -> SmartDashboard.putString("Stage", "Start")),
                    runOnce(() -> swerveDrive.resetOdometry(zoooomToCube.getInitialPose())),
                    runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                    runOnce(() -> swerveDrive.stopModules())
                ),

                //preload
                // claw.intake(),
                // Commands.deadline(
                //     Commands.waitSeconds(2),
                //     Commands.runOnce(() -> SmartDashboard.putString("Stage", "Score")),
                //     Commands.sequence(
                //         Commands.runOnce(() -> arm.setTargetTicks(armPosFinal)),
                //         Commands.waitSeconds(0.5),
                //         Commands.waitUntil(arm.atTargetPosition)
                //     ),
                //     Commands.sequence(
                //         Commands.waitSeconds(0.5),
                //         Commands.runOnce(() -> elevator.setTargetTicks(elevatorPosFinal)),
                //         Commands.waitSeconds(0.5),
                //         Commands.waitUntil(elevator.atTargetPosition)
                //     )
                // // ),
                // Commands.waitSeconds(0.5),
                // // claw.outtake(),
                // claw.setPower(.3),
                // Commands.waitSeconds(0.5),
                // claw.setPowerZero(),
                
                // //stow
                // Commands.deadline(
                //     Commands.waitSeconds(0.5),
                //     Commands.runOnce(() -> SmartDashboard.putString("Stage", "Stow")),
                //     Commands.sequence(
                //         Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow)),
                //         Commands.waitSeconds(0.5),
                //         Commands.waitUntil(elevator.atTargetPosition)
                //     ),
                //     Commands.sequence(
                //         Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmSubstation)),
                //         Commands.waitSeconds(0.5),
                //         Commands.waitUntil(arm.atTargetPosition)
                //     )
                // ),

                //trajectory to cube
                //Commands.runOnce(() -> SmartDashboard.putString("Moved On", "zoomin")),
                zoooomToCubeCommand,
                runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                runOnce(() -> swerveDrive.stopModules()),

                // // new TurnToAngle(170, swerveDrive),
                // runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                // runOnce(() -> swerveDrive.stopModules()),

                Commands.sequence(
                    //Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", true)),
                    Commands.runOnce(() -> vision.initVisionPickupOnGround(OBJECT_TYPE.CUBE)),

                    Commands.race(
                        new RunCommand(() -> vision.driveToCubeOnGround(), arm, elevator, claw, swerveDrive).until(vision.cameraStatusSupplier),
                        Commands.waitSeconds(3) // TODO DEBUG
                    ),
    
                    // Drop arm and elevator so the game piece can be intook
                    // Commands.race(
                    //     Commands.waitSeconds(5),
                    //     Commands.parallel( // End command once both arm and elevator have reached their target position
                    //         Commands.waitUntil(arm.atTargetPosition),
                    //         Commands.waitUntil(elevator.atTargetPosition),
                    //         Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup)),
                    //         Commands.sequence(
                    //             Commands.waitSeconds(0.25),
                    //             Commands.runOnce(() -> elevator.setTargetTicks(-160000))
                    //         )
                    //     )
                    // ),

                    // Open claw/Start claw intake rollers
                    claw.setPower(-0.3),
                    new WaitCommand(.5),
    
                    // // Close claw/stop claw intake rollers/low background rolling to keep control of game piece
                    claw.setPower(-0.15)
                    
                    //Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", false))
                ),

                new TurnToAngle(179.9, swerveDrive),

                cubeToZoooomCommand,
                runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                runOnce(() -> swerveDrive.stopModules()),

                claw.setPower(0.3),
                Commands.waitSeconds(0.5),

                // new TurnToAngle(180, swerveDrive),
                zoooomPartTwoCommand,
                runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                runOnce(() -> swerveDrive.stopModules()),

                new TurnToAngle(0, swerveDrive)

                // //vision pickup
                // // Arm is moved to pick up cube, ends with arm/elev extended and cube in the claw
                // vision.VisionPickupGroundNoArm(OBJECT_TYPE.CUBE)

                // //stow
                // Commands.deadline(
                //     Commands.waitSeconds(2),
                //     Commands.parallel( // End command once both arm and elevator have reached their target position
                //         Commands.waitUntil(arm.atTargetPosition),
                //         Commands.waitUntil(elevator.atTargetPosition),
                //         Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                //         Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow))
                //     )
                // ),

                //trajectory to grid
                //cubeToZoooomCommand,
                //runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                //runOnce(() -> swerveDrive.stopModules())

                // //vision score
                // vision.VisionScore(OBJECT_TYPE.CUBE, SCORE_POS.HIGH),

                // //score
                // Commands.deadline(
                //     Commands.waitSeconds(0.5),
                //     Commands.runOnce(() -> SmartDashboard.putString("Stage", "Score")),
                //     Commands.sequence(
                //         Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorScoreHigh)),
                //         Commands.waitSeconds(0.5),
                //         Commands.waitUntil(elevator.atTargetPosition)
                //     ),
                //     Commands.sequence(
                //         Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScoreCubeHigh)),
                //         Commands.waitSeconds(0.5),
                //         Commands.waitUntil(arm.atTargetPosition)
                //     )
                // ),

            //     //outtake
            //     claw.setPower(0.3),
            //     Commands.waitSeconds(0.5),
            //     claw.setPower(0),

            //     //stow
            //     Commands.deadline(
            //         Commands.waitSeconds(0.5),
            //         Commands.runOnce(() -> SmartDashboard.putString("Stage", "Stow")),
            //         Commands.sequence(
            //             Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow)),
            //             Commands.waitSeconds(0.5),
            //             Commands.waitUntil(elevator.atTargetPosition)
            //         ),
            //         Commands.sequence(
            //             Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
            //             Commands.waitSeconds(0.5),
            //             Commands.waitUntil(arm.atTargetPosition)
            //         )
            //     )
            )
            // run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
            // run(() -> elevator.moveMotionMagic(arm.getArmAngle()))
        );
    }
}
