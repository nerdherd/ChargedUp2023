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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

import static frc.robot.Constants.SwerveAutoConstants.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

public class SwerveAutos {
    public static CommandBase pathplannerAuto(SwerveDrivetrain swerveDrive, Arm arm, Claw claw) {
        PathPlannerTrajectory testPath = PathPlanner.loadPath(
            "Test Path", 
            new PathConstraints(
                kMaxSpeedMetersPerSecond, 
                kMaxAccelerationMetersPerSecondSquared));
        
        HashMap<String, Command> events = new HashMap<>() {{
            //put();
        }};

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            swerveDrive::getPose, 
            swerveDrive::resetOdometry, 
            new PIDConstants(kPXController, kIXController, kDXController), 
            new PIDConstants(kPThetaController, kIThetaController, kDThetaController), 
            SwerveDriveConstants.kDriveKinematics::toSwerveModuleStates, 
            events, 
            swerveDrive);
        
        return autoBuilder.followPathWithEvents(testPath);
    }

    public static CommandBase translateBy(SwerveDrivetrain swerveDrive, double xTranslation, double yTranslation, double angle) {
        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
        // Create Actual Trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(),
            new Pose2d(xTranslation, yTranslation, new Rotation2d(angle)), 
            trajectoryConfig);
        
        //Create PID Controllers
        PIDController xController = new PIDController(kPXController, kIXController, kDXController);
        PIDController yController = new PIDController(kPYController,kIYController, kDYController);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand autoCommand = new SwerveControllerCommand(
            trajectory, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
            xController, yController, thetaController, swerveDrive::setModuleStates, swerveDrive);
        
        return sequence(
            autoCommand,
            runOnce(swerveDrive::stopModules, swerveDrive)
        );
    }

    public enum StartPosition {
        LEFT,
        RIGHT,
        MIDDLE
    }

    public enum ScorePosition {
        HYBRID,
        MID,
        HIGH
    }

    /**
     * Start with the front left swerve module aligned with the charging station's edge
     * in the x axis and around 8 inches to the right in the y axis
     * @param swerveDrive
     * @return
     */
    public static CommandBase onePieceChargeAuto(SwerveDrivetrain swerveDrive, Arm arm, Elevator elevator, Claw claw, StartPosition position, Alliance alliance) {
        // Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
        
        double pickupAngle = 0;
        double chargeYTranslation = 0;
        double pickupXDistance = 0;
        double pickupYDistance = 0;

        Pose2d initialPose = new Pose2d();

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
                initialPose = new Pose2d(1.7, 1, new Rotation2d());
                break;
            case LEFT:
                pickupAngle = 10;
                chargeYTranslation = 1.7;
                pickupXDistance = 4;
                pickupYDistance = 0.25;
                initialPose = new Pose2d(1.7, 4.4, new Rotation2d());
                break;
            case MIDDLE:
                pickupAngle = -20;
                chargeYTranslation = 0;
                pickupXDistance = 5; // TODO: Measure IRL
                pickupYDistance = -0.5;
                initialPose = new Pose2d(1.7, 2.2, new Rotation2d());
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
        
        final Pose2d initialPoseFinal = initialPose;
        
        return parallel(
            run(() -> arm.moveArmMotionMagic(elevator.percentExtended.getAsDouble())),
            run(() -> elevator.moveMotionMagic(arm.armAngle.getAsDouble())),
            sequence(
                        // runOnce(swerveDrive::zeroHeading),
                race(
                    //new TurnToAngle(0, swerveDrive),
                    waitSeconds(1)               
                ),
                runOnce(() -> SmartDashboard.putString("Stage", "Start")),
                runOnce(() -> swerveDrive.resetOdometry(initialPoseFinal)),
                // autoCommand,
                // new TurnToAngle(180, swerveDrive),
                runOnce(() -> swerveDrive.stopModules()),
                race(
                    sequence(
                        runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScore)),
                        waitSeconds(0.5),
                        waitUntil(arm.atTargetPosition)
                    ),
                    waitSeconds(2)
                ),
                // waitSeconds(1),

                runOnce(() -> SmartDashboard.putString("Stage", "Score")),
                // arm.armExtend(),
                race(
                    sequence(
                        runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorScoreHigh)),
                        waitSeconds(0.5),
                        waitUntil(elevator.atTargetPosition)
                    ),
                    waitSeconds(2)
                ),
                waitSeconds(1),

                claw.clawOpen(),
                waitSeconds(1),
                // claw.clawClose(),
                // arm.armStow(),
                race(
                    sequence(
                        runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow)),
                        waitSeconds(0.5),
                        waitUntil(elevator.atTargetPosition)
                    ),
                    waitSeconds(2)
                ),
                waitSeconds(1),

                race(
                    sequence(
                        runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                        waitSeconds(0.5),
                        waitUntil(arm.atTargetPosition)
                    ),
                    waitSeconds(2)
                ),
                // waitSeconds(1),
                runOnce(() -> SmartDashboard.putString("Stage", "Stow")),
                scoreToPickupCommand,
                runOnce(() -> swerveDrive.stopModules()),
                // arm.armExtend(),
                // claw.clawOpen(),
                // waitSeconds(0.5),
                race(
                    // runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGround)),
                    sequence(
                        runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGround)),
                        waitSeconds(0.5),
                        waitUntil(arm.atTargetPosition)
                    ),
                    waitSeconds(2)
                ),
                // claw.clawClose(),
                // waitSeconds(2),

                runOnce(() -> SmartDashboard.putString("Stage", "Ground")),
                claw.clawClose(),
                waitSeconds(1),
                race(
                    runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),

                    waitUntil(arm.atTargetPosition),
                    waitSeconds(2)
                ),
                // waitSeconds(1),

                runOnce(() -> SmartDashboard.putString("Stage", "Stow 2")),
                pickupToScoreCommand,
                // runOnce(() -> swerveDrive.stopModules()),
                // waitSeconds(1),
                // autoCommand4,
                // runOnce(() -> swerveDrive.stopModules()),
                // waitSeconds(1),
                // autoCommand5,
                runOnce(() -> swerveDrive.stopModules()),
                race(
                    waitSeconds(0.5)
                    // new TurnToAngle(180, swerveDrive)                
                ),
                
                race(
                    runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScore)),

                    waitUntil(arm.atTargetPosition),
                    waitSeconds(2)
                ),
                
                runOnce(() -> SmartDashboard.putString("Stage", "Score 2")),
                // arm.armExtend(),
                race(
                    sequence(
                        runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorScoreHigh)),
                        waitSeconds(0.5),
                        waitUntil(elevator.atTargetPosition)
                    ),
                    waitSeconds(2)
                ),
                waitSeconds(1),

                claw.clawOpen(),
                waitSeconds(1),
                
                // arm.armStow(),
                race(
                    sequence(
                        runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow)),
                        waitSeconds(0.5),
                        waitUntil(elevator.atTargetPosition)
                    ),
                    waitSeconds(2)
                ),
                waitSeconds(1),

                race(
                    runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),

                    waitUntil(arm.atTargetPosition),
                    waitSeconds(2)
                ),
                waitSeconds(1),

                runOnce(() -> SmartDashboard.putString("Stage", "Stow 3")),
                scoreToChargeCommand,
                // new TheGreatBalancingAct(swerveDrive),
                new TimedBalancingAct(swerveDrive, 0.5, SwerveAutoConstants.kPBalancingInitial, SwerveAutoConstants.kPBalancing),
                runOnce(() -> swerveDrive.stopModules()))
            );
    } 

    public static CommandBase preloadChargeAuto(SwerveDrivetrain swerveDrive, Arm arm, Elevator elevator, Claw claw, StartPosition startPos, ScorePosition scorePos, double waitTime, boolean goAround) {
        return sequence(
            race(
                sequence(
                    runOnce(() -> arm.setTargetTicks(ArmConstants.kArmScore)),
                    waitSeconds(0.5),
                    waitUntil(arm.atTargetPosition)
                ),
                waitSeconds(2)
            ),
            race(
                sequence(
                    runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorScoreHigh)),
                    waitSeconds(0.5),
                    waitUntil(elevator.atTargetPosition)
                ),
                waitSeconds(2)
            ),
            waitSeconds(1),
            claw.clawOpen(),
            chargeAuto(swerveDrive, startPos, waitTime, goAround)
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
                yOvershoot = 2;
                break;
            case RIGHT:
                yTranslation = -1.75;
                yOvershoot = -2;
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
                new Pose2d(0, 0, new Rotation2d(0)), 
                List.of(
                    new Translation2d(0.25, yOvershoot)), 
                new Pose2d(1.5, yTranslation, Rotation2d.fromDegrees(0)), 
                trajectoryConfig);
        } else {
            trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)), 
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
     * @param swerveDrive 
     * @return Command to reset odometry run auto to go onto charging station then run balancing auto
     */
    public static CommandBase chargeAuto(SwerveDrivetrain swerveDrive) {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
        
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                new Translation2d(-0.5, 0),
                new Translation2d(-0.5, -2)), 
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
            autoCommand,
            new TimedBalancingAct(swerveDrive, 0.5, 
                SwerveAutoConstants.kPBalancingInitial, 
                SwerveAutoConstants.kPBalancing)
            // new TheGreatBalancingAct(swerveDrive),
            // new TowSwerve(swerveDrive)
        );
    }
}
