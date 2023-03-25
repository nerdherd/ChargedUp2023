package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.claw.MotorClaw;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.util.NerdyMath;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.SwerveAutoConstants.*;

public class ChargeAutos {
    public static CommandBase preloadHighChargeMiddle(SwerveDrivetrain swerveDrive, Arm arm, Elevator elevator, MotorClaw claw) {
        CommandBase auto = sequence(
            preloadHigh(arm, elevator, claw),
            deadline(
                chargeMiddle(swerveDrive),
                run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
                run(() -> elevator.moveMotionMagic(arm.getArmAngle()))
            )
        ).finallyDo((x) -> swerveDrive.getImu().setOffset(180));

        auto.setName("Preload High Charge Middle");
        
        return auto;
    }

    public static CommandBase preloadHighChargeTaxiMiddle(SwerveDrivetrain swerveDrive, Arm arm, Elevator elevator, MotorClaw claw) {
        CommandBase auto = sequence(
            preloadHigh(arm, elevator, claw),
            deadline(
                chargeTaxiMiddle(swerveDrive),
                run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
                run(() -> elevator.moveMotionMagic(arm.getArmAngle()))
            )
        ).finallyDo((x) -> swerveDrive.getImu().setOffset(180));

        auto.setName("Preload High Charge Taxi Middle");
        
        return auto;
    }

    public static CommandBase customPreloadHighChargeTaxiMiddle(SwerveDrivetrain swerveDrive, Arm arm, Elevator elevator, MotorClaw claw) {
        CommandBase auto = sequence(
            preloadHigh(arm, elevator, claw),
            deadline(
                customChargeTaxiMiddle(swerveDrive),
                run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
                run(() -> elevator.moveMotionMagic(arm.getArmAngle()))
            )
        ).finallyDo((x) -> swerveDrive.getImu().setOffset(180));

        auto.setName("Custom Preload High Charge Taxi Middle");
        
        return auto;
    }

    public static CommandBase preloadHigh(Arm arm, Elevator elevator, MotorClaw claw) {
        return race(
            waitSeconds(5),
            sequence(
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

    public static CommandBase chargeMiddle(SwerveDrivetrain swerveDrive) {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kChargeSpeedMetersPerSecond, 
            kChargeAccelerationMetersPerSecondSquared);
        
        Trajectory chargeTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(-0.125, 0, new Rotation2d(0)), 
            List.of(
                new Translation2d(-0.25, 0),
                new Translation2d(-0.25, 0)), 
            new Pose2d(-3, -0.01, Rotation2d.fromDegrees(0)),
            trajectoryConfig);
        
        PIDController xController = new PIDController(kPXController, kIXController, kDXController);
        PIDController yController = new PIDController(kPYController, kIYController, kDYController);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand chargeCommand = new SwerveControllerCommand(
            chargeTrajectory, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return sequence(
            runOnce(() -> swerveDrive.resetOdometry(chargeTrajectory.getInitialPose())),
            chargeCommand,
            new TheGreatBalancingAct(swerveDrive)
        );
    }

    public static CommandBase chargeTaxiMiddle(SwerveDrivetrain swerveDrive) {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kChargeSpeedMetersPerSecond, 
            kChargeAccelerationMetersPerSecondSquared);

        Trajectory goPastCharge = TrajectoryGenerator.generateTrajectory(
            new Pose2d(-0.125, 0, new Rotation2d(0)), 
            List.of(
                new Translation2d(-0.25, 0),
                new Translation2d(-0.25, 0)), 
            new Pose2d(-5, -0.01, Rotation2d.fromDegrees(0)),
            trajectoryConfig);
        
        Trajectory returnToCharge = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(-5, 0.01, Rotation2d.fromDegrees(0)),
                new Pose2d(-2, -0.01, Rotation2d.fromDegrees(0))
            ),
            trajectoryConfig);
        
        PIDController xController = new PIDController(kPXController, kIXController, kDXController);
        PIDController yController = new PIDController(kPYController, kIYController, kDYController);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand goPastChargeCommand = new SwerveControllerCommand(
            goPastCharge, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        SwerveControllerCommand returnToChargeCommand = new SwerveControllerCommand(
            returnToCharge, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return sequence(
            runOnce(() -> swerveDrive.resetOdometry(goPastCharge.getInitialPose())),
            race(
                waitSeconds(6),
                goPastChargeCommand,
                // Wait until it's within 5 degrees
                sequence(
                    waitSeconds(2),
                    waitUntil(
                        () -> {
                            boolean success = NerdyMath.inRange(
                                swerveDrive.getImu().getRotation3d().getX(),
                                -10,
                                0
                            );
                            SmartDashboard.putBoolean("Stop charge", success);
                            return success;
                        }
                    )
                )
            ),
            waitSeconds(0.1),
            parallel(
                runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates), swerveDrive),
                runOnce(() -> swerveDrive.resetOdometry(new Pose2d(-4.5, 0, new Rotation2d())))
            ),
            waitSeconds(1.5),
            returnToChargeCommand,
            new TheGreatBalancingAct(swerveDrive)
        );
    }

    public static CommandBase customChargeTaxiMiddle(SwerveDrivetrain swerveDrive) {
        SlewRateLimiter speedLimiter = new SlewRateLimiter(kChargeAccelerationMetersPerSecondSquared);

        return sequence(
            runOnce(() -> swerveDrive.resetOdometry(new Pose2d())),
            race(
                waitSeconds(6),
                run(() -> {
                    swerveDrive.setModuleStates(
                        SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                speedLimiter.calculate(-kChargeSpeedMetersPerSecond), 0, 0,
                                swerveDrive.getImu().getRotation2d())
                        )
                    );
                }),
                // Wait until it's within 10 degrees (going downwards)
                sequence(
                    waitSeconds(2),
                    waitUntil(
                        () -> {
                            boolean success = NerdyMath.inRange(
                                swerveDrive.getImu().getRotation3d().getX(),
                                -10, 0
                            );
                            SmartDashboard.putBoolean("Stop charge", success);
                            return success;
                        }
                    )
                )
            ),
            // Slow down
            deadline(
                waitSeconds(0.1),
                run(() -> {
                    swerveDrive.setModuleStates(
                        SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                speedLimiter.calculate(0), 0, 0,
                                swerveDrive.getImu().getRotation2d())
                        )
                    );
                })
            ),
            parallel(
                runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates), swerveDrive),
                runOnce(() -> swerveDrive.resetOdometry(new Pose2d(-3.5, 0, new Rotation2d()))),
                runOnce(() -> speedLimiter.reset(0))
            ),
            runOnce(() -> swerveDrive.stopModules()),
            // Wait for charge station to flatten out
            waitSeconds(1.5),
            race(
                waitSeconds(4),
                run(() -> {
                    swerveDrive.setModuleStates(
                        SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                speedLimiter.calculate(kChargeSpeedMetersPerSecond), 0, 0,
                                swerveDrive.getImu().getRotation2d())
                        )
                    );
                }),
                waitUntil(() -> swerveDrive.getPose().getX() > -1)
            ),
            new TheGreatBalancingAct(swerveDrive)
        );
        // .finallyDo((x) -> swerveDrive.resetOdometry(new Pose2d(3.88, 3.48, new Rotation2d(180))));
    }
}
