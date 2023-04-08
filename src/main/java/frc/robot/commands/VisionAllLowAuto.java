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

        double zoooomAllianceThingy = 1.0;
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
                new Pose2d(3.6, 0.2, Rotation2d.fromDegrees(179.9)),
                new Pose2d(0.8, 0.2, Rotation2d.fromDegrees(179.9)),
                // new Pose2d(-0.8, -1.0, Rotation2d.fromDegrees(0)),
                new Pose2d(0.2, 1.0, Rotation2d.fromDegrees(179.9))
            ),
            trajectoryConfig);

        Trajectory zoooomPartTwo = TrajectoryGenerator.generateTrajectory(
            List.of(
                //new Pose2d(0.2, 1.0, Rotation2d.fromDegrees(179.9)),
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
                Commands.parallel(
                    Commands.runOnce(() -> SmartDashboard.putString("Stage", "Start")),
                    Commands.runOnce(() -> swerveDrive.resetOdometry(zoooomToCube.getInitialPose()))
                    //runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)), // TBD
                    //runOnce(() -> swerveDrive.stopModules())
                ),

                //trajectory to cube
                //Commands.runOnce(() -> SmartDashboard.putString("Moved On", "zoomin")),
                Commands.parallel(
                    zoooomToCubeCommand/* ,
    
                    //Drop arm to half way
                    Commands.race(
                        Commands.waitSeconds(5),
                        Commands.parallel( // End command once both arm and elevator have reached their target position
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup)) // TBD, half way
                            Commands.sequence(
                                Commands.waitSeconds(0.25),
                                Commands.runOnce(() -> elevator.setTargetTicks(-160000))
                            )
                        )
                    )*/

                    // TODO to test which one will fast!!!!
                ),
                Commands.runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                Commands.runOnce(() -> swerveDrive.stopModules()),


                // // new TurnToAngle(170, swerveDrive),
                // runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                // runOnce(() -> swerveDrive.stopModules()),

                Commands.parallel(
                    Commands.sequence(
                        Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", true)),
                        Commands.runOnce(() -> vision.initVisionPickupOnGround(OBJECT_TYPE.CUBE)),

                        Commands.race(
                            new RunCommand(() -> vision.driveToCubeOnGround(claw, 5), arm, elevator, claw, swerveDrive).until(vision.cameraStatusSupplier),
                            Commands.waitSeconds(20) // TODO DEBUG
                            // TODO need add protection here!!!!!!
                        ),
                
                        // TODO: low prio.... return the command if exception?
                        Commands.runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                        Commands.runOnce(() -> swerveDrive.stopModules())

                    ) // TODO: GET RID OF TEMPORARy
                    ),/* ,
                    
    
            //         //Drop arm and pick cube up
            //         Commands.race(
            //             Commands.waitSeconds(5),
            //             Commands.parallel( // End command once both arm and elevator have reached their target position
            //                 Commands.waitUntil(arm.atTargetPosition),
            //                 Commands.waitUntil(elevator.atTargetPosition),
            //                 Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup)),
            //                 Commands.sequence(
            //                     Commands.waitSeconds(0.25),
            //                     Commands.runOnce(() -> elevator.setTargetTicks(-160000))
            //                 )
            //             )
            //         )*/
            //         // TODO to test which one will be fast!!!!
            //     ),

                // Open claw/Start claw intake rollers
                // claw.setPower(-0.3),
                // new WaitCommand(.5),
                    
                Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", false)),
                
                Commands.parallel(
                    // Close claw/stop claw intake rollers/low background rolling to keep control of game piece
                // claw.setPower(-0.15),

                    Commands.sequence(
                        new TurnToAngle(179.9, swerveDrive),

                        cubeToZoooomCommand
                    )
                ),
                    /* ,

                

            //         // TODO a lot tuning
    
            //         //up arm to score position
            //         Commands.race(
            //             Commands.waitSeconds(5),
            //             Commands.parallel( // End command once both arm and elevator have reached their target position
            //                 Commands.waitUntil(arm.atTargetPosition),
            //                 Commands.waitUntil(elevator.atTargetPosition),
            //                 Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup)),
            //                 Commands.sequence(
            //                     Commands.waitSeconds(0.25),
            //                     Commands.runOnce(() -> elevator.setTargetTicks(-160000))
            //                 )
            //             )
            //         )*/
            //     ),

            //     // TODO: apriltag?
                Commands.runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                Commands.runOnce(() -> swerveDrive.stopModules()),

            //     claw.setPower(0.3),
            //     Commands.waitSeconds(0.5),
            //     claw.setPower(0),

                Commands.parallel(
                    zoooomPartTwoCommand/* ,
    
                    //Drop arm to half way
                    Commands.race(
                        Commands.waitSeconds(5),
                        Commands.parallel( // End command once both arm and elevator have reached their target position
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup)) // TBD, half way
                            Commands.sequence(
                                Commands.waitSeconds(0.25),
                                Commands.runOnce(() -> elevator.setTargetTicks(-160000))
                            )
                        )
                    )*/
                ),
                Commands.runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                Commands.runOnce(() -> swerveDrive.stopModules()),


                new TurnToAngle(0, swerveDrive),

                Commands.waitSeconds(1000)


            //     // vision pickup
            //     Commands.parallel(
            //         Commands.sequence(
            //             //Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", true)),
            //             Commands.runOnce(() -> vision.initVisionPickupOnGround(OBJECT_TYPE.CUBE)),

            //             Commands.race(
            //                 new RunCommand(() -> vision.driveToCubeOnGround(), arm, elevator, claw, swerveDrive).until(vision.cameraStatusSupplier),
            //                 Commands.waitSeconds(5) // TODO DEBUG
            //                 // TODO need add protection here!!!!!!
            //             ),

            //             Commands.runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
            //             Commands.runOnce(() -> swerveDrive.stopModules())
            //         )/* ,
    
            //         //Drop arm and pick cube up
            //         Commands.race(
            //             Commands.waitSeconds(5),
            //             Commands.parallel( // End command once both arm and elevator have reached their target position
            //                 Commands.waitUntil(arm.atTargetPosition),
            //                 Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup))
            //             )
            //         )*/
            //     ),

            //     // Open claw/Start claw intake rollers
            //     claw.setPower(-0.3),
            //     new WaitCommand(.5),
            //     // // Close claw/stop claw intake rollers/low background rolling to keep control of game piece
            //     claw.setPower(-0.15),

            //     new TurnToAngle(179.9, swerveDrive)
                
            //     //Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", false))
                
            // )
            // // run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
            // // run(() -> elevator.moveMotionMagic(arm.getArmAngle()))

            
        ))
        ;
    }
}
