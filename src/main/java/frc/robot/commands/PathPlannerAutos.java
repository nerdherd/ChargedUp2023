package frc.robot.commands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

import static frc.robot.Constants.SwerveAutoConstants.*;

public class PathPlannerAutos {
    public static CommandBase pathplannerAuto(String pathName, SwerveDrivetrain swerveDrive) {
        PathPlannerTrajectory path = PathPlanner.loadPath(
            pathName, 
            new PathConstraints(
                1, 
                1));
        
        HashMap<String, Command> events = new HashMap<>() {{
            //put();
        }};
    
        List<State> states = path.getStates();
        for (int i = 0; i<states.size(); i++) {
            SmartDashboard.putString("State #" + i, states.get(i).toString());
        }

        // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        //     swerveDrive::getPose, 
        //     swerveDrive::resetOdometry, 
        //     SwerveDriveConstants.kDriveKinematics,
        //     new PIDConstants(kPXController, kIXController, kDXController), 
        //     new PIDConstants(kPThetaController, kIThetaController, kDThetaController), 
        //     swerveDrive::setModuleStates,
        //     events, 
        //     true,
        //     swerveDrive);
        
        PIDController xController = new PIDController(kPXController, kIXController, kDXController);
        PIDController yController = new PIDController(kPYController, kIYController, kDYController);
        PIDController thetaController = new PIDController(kPThetaController, kIThetaController, kDThetaController);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        PPSwerveControllerCommand autoCommand = new PPSwerveControllerCommand(
            path, 
            swerveDrive::getPose, 
            SwerveDriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            swerveDrive::setModuleStates,
            true,
            swerveDrive);

        return Commands.sequence(
            // autoBuilder.followPathWithEvents(testPath)
            // Get rid of this once we get real odometry
            Commands.runOnce(() -> swerveDrive.getImu().zeroAll()),
            Commands.runOnce(() -> swerveDrive.setPoseMeters(path.getInitialPose())),
            autoCommand
        );
    }
}
