package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

import static frc.robot.Constants.SwerveAutoConstants.*;
import static frc.robot.Constants.PathPlannerConstants.*;

public class PathPlannerAutos {
    private static HashMap<String, PathPlannerTrajectory> cachedPaths = new HashMap<>();

    /**
     * Load the selected path from storage.
     * @param pathName
     */
    public static void initPath(String pathName) {
        if (cachedPaths.containsKey(pathName)) {
            DriverStation.reportWarning(String.format("Path '%s' has been loaded more than once.", pathName), true);
        }

        PathPlannerTrajectory path = PathPlanner.loadPath(pathName, kPPPathConstraints);

        if (path == null) {
            DriverStation.reportWarning(String.format("Path '%s' could not be loaded!", pathName), true);
        }
        cachedPaths.put(pathName, path);
    }

    /**
     * Create an auto with the selected PathPlanner path.
     * @param pathName
     * @param swerveDrive
     * @return
     */
    public static CommandBase pathplannerAuto(String pathName, SwerveDrivetrain swerveDrive) {
        if (!cachedPaths.containsKey(pathName)) {
            DriverStation.reportWarning(String.format("Path '%s' was not pre-loaded into memory, which may cause lag during the Autonomous Period.", pathName), true);
            initPath(pathName);
        }

        PathPlannerTrajectory path = cachedPaths.get(pathName);
        
        HashMap<String, Command> events = new HashMap<>() {{
            // TODO: Add commands here 
            // put("Command name", Command);
        }};
        
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
            Commands.runOnce(() -> swerveDrive.getImu().zeroAll()),
            // TODO: Once we get real odometry with vision, get rid of this
            Commands.runOnce(() -> swerveDrive.setPoseMeters(path.getInitialPose())),
            autoCommand
        );
    }
}
