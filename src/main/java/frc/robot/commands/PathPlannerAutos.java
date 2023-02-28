package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
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

public class PathPlannerAutos {
    public static CommandBase pathplannerAuto(String pathName, SwerveDrivetrain swerveDrive, Elevator elevator, Arm arm, MotorClaw claw, boolean preload) {
        PathPlannerTrajectory testPath = PathPlanner.loadPath(
            pathName, 
            new PathConstraints(
                kMaxSpeedMetersPerSecond, 
                kMaxAccelerationMetersPerSecondSquared));
        
        HashMap<String, Command> events = new HashMap<>() {{
            put("Pickup", sequence(
                deadline(
                    waitSeconds(2),
                    sequence(
                        runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup)),
                        waitSeconds(0.5),
                        waitUntil(arm.atTargetPosition)
                    ),
                    sequence(
                        runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow)),
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
                )
            ));
            put("Score", sequence(
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
                )
            ));
            put("Charge", new TheGreatBalancingAct(swerveDrive));

            if (preload) {
                put("ScorePickup", get("Score"));
            } else {
                put("ScorePickup", none());
            }
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

    public static CommandBase pathplannerAuto(String pathName, SwerveDrivetrain swerveDrive, Elevator elevator, Arm arm, MotorClaw claw) {
        return pathplannerAuto(pathName, swerveDrive, elevator, arm, claw, false);
    }

    public static CommandBase visionPathPlannerAuto(String pathName, SwerveDrivetrain swerveDrive, MotorClaw claw, VROOOOM vision, boolean preload) {
        PathPlannerTrajectory testPath = PathPlanner.loadPath(
            pathName, 
            new PathConstraints(
                kMaxSpeedMetersPerSecond, 
                kMaxAccelerationMetersPerSecondSquared));
        
        HashMap<String, Command> events = new HashMap<>() {{
            put("Pickup", sequence(
                vision.updateCurrentGameObject(OBJECT_TYPE.CONE),
                vision.updateCurrentHeight(SCORE_POS.MID),
                vision.VisionPickup()
            ));
            put("Score", sequence(
                vision.updateCurrentGameObject(OBJECT_TYPE.CONE),
                vision.updateCurrentHeight(SCORE_POS.MID),
                vision.VisionScore()
            ));
            put("Charge", new TheGreatBalancingAct(swerveDrive));
            if (preload) {
                put("ScorePickup", get("Score"));
            } else {
                put("ScorePickup", none());
            }
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
}
