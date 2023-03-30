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

public class VisionCablesidePickup {
    
    /**
     * You should not be calling this function directly (nothing will break if you do, 
     * but it is meant to be paired with vision auto choosing). Instead, call visionAutoChoose below.
     * 
     * @return Auto Command
     */
    public static CommandBase visionPreloadPickupScore(SwerveDrivetrain swerveDrive, VROOOOM vision, Arm arm, Elevator elevator, MotorClaw claw, 
        Alliance alliance) {
        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            SwerveAutoConstants.kMaxSpeedMetersPerSecond, SwerveAutoConstants.kMaxAccelerationMetersPerSecondSquared);

        double zoooomAllianceThingy = 1.0;;
        if (alliance == Alliance.Red) {
            zoooomAllianceThingy = -1.0;
        }

        Trajectory scoreToPickup = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                new Translation2d(-0.2, 0.2) // only for blue side right now
            ),
            new Pose2d(-3.6, 0.2, Rotation2d.fromDegrees(0)),
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

        final Alliance allianceFinal = alliance;

        return race(
            Commands.waitSeconds(15), // TODO DEL

            sequence(
                Commands.parallel(
                    //runOnce(() -> SmartDashboard.putString("Stage", "Start")),
                    runOnce(() -> swerveDrive.resetOdometry(scoreToPickup.getInitialPose()))
                    //runOnce(() -> swerveDrive.stopModules())
                ),

                // Move arm and elevator, arm is moved 0.5 seconds after the elevator to prevent power chain from getting caught
                /*Commands.race(
                    Commands.waitSeconds(5), 
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
                waitSeconds(0.25),*/

                // Open claw/eject piece with rollers
                claw.setPower(0.5),
                // Wait to outtake
                Commands.waitSeconds(.5),

                Commands.parallel(
                    // Close claw/stop rollers
                    claw.setPower(0),

                    // moving arm/elev to drive mode
                    /*Commands.deadline(
                        Commands.waitSeconds(3) ,
                        Commands.parallel( // End command once both arm and elevator have reached their target position
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                            Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow))
                        )
                    ),*/

                    // travel to pick up spot
                    Commands.deadline( // TODO: need to add protection here!!!!!!
                        Commands.waitSeconds(6) ,
                        scoreToPickupCommand
                    )
                ),

                runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
                runOnce(() -> swerveDrive.stopModules()),
                new TurnToAngle(179.9, swerveDrive),

                Commands.parallel(
                    Commands.sequence(
                        //Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", true)),
                        Commands.runOnce(() -> vision.initVisionPickupOnGround(OBJECT_TYPE.CUBE)),

                        Commands.race(
                            new RunCommand(() -> vision.driveToCubeOnGround(), arm, elevator, claw, swerveDrive).until(vision.cameraStatusSupplier),
                            Commands.waitSeconds(3) // TODO DEBUG
                        )
                    )/* ,
    
                    //Drop arm and pick cube up
                    Commands.race(
                        Commands.waitSeconds(5),
                        Commands.parallel( // End command once both arm and elevator have reached their target position
                            Commands.waitUntil(arm.atTargetPosition),
                            Commands.waitUntil(elevator.atTargetPosition),
                            Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup)),
                            Commands.sequence(
                                Commands.waitSeconds(0.25),
                                Commands.runOnce(() -> elevator.setTargetTicks(-160000))
                            )
                        )
                    )*/
                ),
                
                // Open claw/Start claw intake rollers
                claw.setPower(-0.3),
                new WaitCommand(.5),    
                // // Close claw/stop claw intake rollers/low background rolling to keep control of game piece
                claw.setPower(-0.15),
                
                // move arm to driving mode
                /*Commands.deadline(
                    Commands.waitSeconds(2) ,
                    Commands.parallel( // End command once both arm and elevator have reached their target position
                        Commands.waitUntil(arm.atTargetPosition),
                        Commands.waitUntil(elevator.atTargetPosition),
                        Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                        Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow))
                    )
                ),*/

                // turn to score angle
                new TurnToAngle(0, swerveDrive)
            )/* ,
            
            run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
            run(() -> elevator.moveMotionMagic(arm.getArmAngle()))*/
        );
    }
}
