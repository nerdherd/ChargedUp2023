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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.claw.MotorClaw;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.util.NerdyMath;

import static frc.robot.Constants.SwerveAutoConstants.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class TestAutos {
    public static CommandBase preloadTaxiChargeBackwardsSLOW(SwerveDrivetrain swerveDrive, MotorClaw claw, Arm arm, Elevator elevator) {
        return sequence(
            deadline(
                waitSeconds(13.5), 
                sequence(
                    ChargeAutos.preloadHigh(arm, elevator, claw),
                    deadline(
                        taxiChargeBackwardsSLOW(swerveDrive),
                        run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
                        run(() -> elevator.moveMotionMagic(arm.getArmAngle()))
                    )    
                )
            ),
            runOnce(() -> swerveDrive.towModules()),
            waitSeconds(0.2),
            runOnce(() -> swerveDrive.stopModules())
        ).finallyDo((x) -> swerveDrive.getImu().setOffset(180));
    }

    public static CommandBase taxiChargeBackwardsSLOW(SwerveDrivetrain swerveDrive, MotorClaw claw, Arm arm, Elevator elevator) {
        return sequence(
            // ChargeAutos.preloadHigh(arm, elevator, claw),
            runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
            deadline(
                taxiChargeBackwardsSLOW(swerveDrive),
                run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
                run(() -> elevator.moveMotionMagic(arm.getArmAngle()))
            )
        );
    }

    public static CommandBase chargeBackwardsSLOW(SwerveDrivetrain swerveDrive, MotorClaw claw, Arm arm, Elevator elevator) {
        return sequence(
            runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
            deadline(
                chargeBackwardsSLOW(swerveDrive),
                run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
                run(() -> elevator.moveMotionMagic(arm.getArmAngle()))
            )
        );
    }

    public static CommandBase preloadChargeBackwardsSLOW(SwerveDrivetrain swerveDrive, MotorClaw claw, Arm arm, Elevator elevator) {
        return sequence(
            ChargeAutos.preloadHigh(arm, elevator, claw),
            deadline(
                chargeBackwardsSLOW(swerveDrive),
                run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
                run(() -> elevator.moveMotionMagic(arm.getArmAngle()))
            )
        ).finallyDo((x) -> swerveDrive.getImu().setOffset(180));
    }

    public static CommandBase taxiChargeBackwardsSLOW(SwerveDrivetrain swerveDrive) {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            1, 
            1);
        
        TrajectoryConfig trajectoryConfig2 = new TrajectoryConfig(
            1.2, 
            1);
        
        trajectoryConfig2.setEndVelocity(1.2);

        // Trajectory goForward = TrajectoryGenerator.generateTrajectory(
        //     List.of(
        //         new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
        //         // new Pose2d(3.5, 0, Rotation2d.fromDegrees(0)),
        //         new Pose2d(3.5, 0.01, Rotation2d.fromDegrees(180))
        //         // new Pose2d(2.5, 0.01, Rotation2d.fromDegrees(180))
        //     ),
        //     trajectoryConfig);
        // Trajectory goBackward = TrajectoryGenerator.generateTrajectory(
        //     List.of(
        //         new Pose2d(3.5, 0, Rotation2d.fromDegrees(180)),
        //         new Pose2d(2, 0.01, Rotation2d.fromDegrees(180))
        //     ),
        //     trajectoryConfig2);
        
        Trajectory fullTraj = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
                new Pose2d(3.65 + 0.25, 0.01, Rotation2d.fromDegrees(180)),
                new Pose2d(3.65 + 0.25, 0.1, Rotation2d.fromDegrees(180)),
                new Pose2d(2 - 0.1, 0.1, Rotation2d.fromDegrees(180))
            ),
            trajectoryConfig2
        );
        // Trajectory comeBack = TrajectoryGenerator.generateTrajectory(
        //     List.of(
        //         new Pose2d(4, 0, Rotation2d.fromDegrees(0)),
        //         new Pose2d(2, 0.01, Rotation2d.fromDegrees(0))
        //     ),
        //     trajectoryConfig);
        
        PIDController xController = new PIDController(kPXController, kIXController, kDXController);
        PIDController yController = new PIDController(kPYController, kIYController, kDYController);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // SwerveControllerCommand goForwardCommand = new SwerveControllerCommand(
        //     goForward, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
        //     xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        // SwerveControllerCommand comeBackCommand = new SwerveControllerCommand(
        //     goBackward, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
        //     xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        SwerveControllerCommand fullCommand = new SwerveControllerCommand(
            fullTraj, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return sequence(
            parallel(
                runOnce(() -> swerveDrive.resetOdometry(fullTraj.getInitialPose())),
                runOnce(() -> swerveDrive.setVelocityControl(true)),
                runOnce(() -> SmartDashboard.putString("Stage", "Going forward")),
                runOnce(() -> swerveDrive.getImu().zeroRoll()),
                runOnce(() -> swerveDrive.getImu().zeroPitch())
            ),
            // goForwardCommand,
            // runOnce(() -> swerveDrive.stopModules()),
            // comeBackCommand,
            fullCommand,
            runOnce(() -> SmartDashboard.putString("Stage", "Balancing")),
            race(
                // waitSeconds(3),
                new TheGreatBalancingAct(swerveDrive, 1.1, 0.0, 0.05, 0.6, 0.0, 0.0)
                // sequence(
                //     waitSeconds(2),
                //     waitUntil(() -> NerdyMath.inRange(swerveDrive.getImu().getRoll(), -1, 1))
                // )
            ),
            runOnce(() -> swerveDrive.towModules()),
            waitSeconds(0.2),
            runOnce(() -> swerveDrive.stopModules())
        );
    }

    public static CommandBase chargeBackwardsSLOW(SwerveDrivetrain swerveDrive) {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            1, 
            2);
        
        trajectoryConfig = trajectoryConfig.setEndVelocity(1);

        Trajectory goForward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
            List.of(
                new Translation2d(1, 0),
                new Translation2d(1.5, 0)
            ),
            new Pose2d(2.5, 0.01, Rotation2d.fromDegrees(180)),
            trajectoryConfig);
        
        PIDController xController = new PIDController(kPXController, kIXController, kDXController);
        PIDController yController = new PIDController(kPYController, kIYController, kDYController);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand goForwardCommand = new SwerveControllerCommand(
            goForward, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return sequence(
            parallel(
                runOnce(() -> swerveDrive.resetOdometry(goForward.getInitialPose())),
                runOnce(() -> swerveDrive.setVelocityControl(true)),
                runOnce(() -> SmartDashboard.putString("Stage", "Going forward")),
                runOnce(() -> swerveDrive.getImu().zeroRoll()),
                runOnce(() -> swerveDrive.getImu().zeroPitch())
            ),
            goForwardCommand,
            runOnce(() -> SmartDashboard.putString("Stage", "Balancing")),
            race(
                // waitSeconds(3),
                new TheGreatBalancingAct(swerveDrive, 0.8, 0.0, 0.05, 0.6, 0.0, 0.0),
                sequence(
                    waitSeconds(2),
                    waitUntil(() -> NerdyMath.inRange(swerveDrive.getImu().getRoll(), -1, 1))
                )
            ),
            runOnce(() -> swerveDrive.towModules()),
            waitSeconds(0.1),
            runOnce(() -> swerveDrive.stopModules())
        );
    }
}
