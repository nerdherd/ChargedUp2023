package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.claw.PistonClaw;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

import static frc.robot.Constants.SwerveAutoConstants.*;

public class PathPlannerAutos {
    public static CommandBase pathplannerAuto(String pathName, SwerveDrivetrain swerveDrive, Arm arm, PistonClaw claw) {
        PathPlannerTrajectory testPath = PathPlanner.loadPath(
            pathName, 
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
}
