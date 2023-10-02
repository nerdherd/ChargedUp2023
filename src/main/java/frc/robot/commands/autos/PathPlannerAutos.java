package frc.robot.commands.autos;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

import static frc.robot.Constants.PathPlannerConstants.*;

public class PathPlannerAutos {
    private static HashMap<String, PathPlannerTrajectory> cachedPaths = new HashMap<>();
    private static HashMap<String, List<PathPlannerTrajectory>> cachedPathGroups = new HashMap<>();
    private static HashMap<String, Command> events = new HashMap<>();

    public static SwerveAutoBuilder autoBuilder;

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

    public static void initPathGroup(String pathName) {
        if (cachedPathGroups.containsKey(pathName)) {
            DriverStation.reportWarning(String.format("Path group '%s' has been loaded more than once.", pathName), true);
        }

        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup(pathName, kPPPathConstraints);

        if (path == null || path.size() == 0) {
            DriverStation.reportWarning(String.format("Path '%s' could not be loaded!", pathName), true);
        }
        cachedPathGroups.put(pathName, path);
    }

    public static List<PathPlannerTrajectory> getPathGroup(String pathNameString) {
        if (!cachedPathGroups.containsKey(pathNameString)) {
            DriverStation.reportWarning(String.format("Path '%s' was not pre-loaded into memory, which may cause lag during the Autonomous Period.", pathNameString), true);
            initPathGroup(pathNameString);
        }
        return cachedPathGroups.get(pathNameString);
        
    }
    
    public static PathPlannerTrajectory getPath(String pathNameString) {
        if (!cachedPaths.containsKey(pathNameString)) {
            DriverStation.reportWarning(String.format("Path '%s' was not pre-loaded into memory, which may cause lag during the Autonomous Period.", pathNameString), true);
            initPath(pathNameString);
        }
        return cachedPaths.get(pathNameString);
        
    }

    public static void init(SwerveDrivetrain swerveDrive) {
        autoBuilder = new SwerveAutoBuilder(
            swerveDrive::getPose, 
            swerveDrive::resetOdometry, 
            PathPlannerConstants.kPPTranslationPIDConstants,
            PathPlannerConstants.kPPRotationPIDConstants,
            swerveDrive::setChassisSpeeds, 
            events,
            swerveDrive);
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
        
        
        // Note: The reason why the commands are manually constructed instead of
        // using SwerveAutoBuilder is because SwerveAutoBuilder doesn't
        // allow doing enableContinuousInput on the rotation PID controller,
        // which may cause issues.

        // Define PID Controllers
        // PIDController xController = new PIDController(kPP_P, kPP_I, kPP_D);
        // PIDController yController = new PIDController(kPP_P, kPP_I, kPP_D);
        // PIDController thetaController = new PIDController(kPP_ThetaP, kPP_ThetaI, kPP_ThetaD);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // // Path following command
        // PPSwerveControllerCommand pathCommand = new PPSwerveControllerCommand(
        //     path, 
        //     swerveDrive::getPose, 
        //     SwerveDriveConstants.kDriveKinematics,
        //     xController,
        //     yController,
        //     thetaController,
        //     swerveDrive::setModuleStates,
        //     true,
        //     swerveDrive);

        // // Follow the path with events
        // FollowPathWithEvents autoCommand = new FollowPathWithEvents(
        //     pathCommand,
        //     path.getMarkers(),
        //     events
        // );

        // Pose2d initialPose2d = path.getInitialPose();
        // if (DriverStation.getAlliance() == Alliance.Red) {
        //     // Flip x value and turn the robot around
        //     double x = initialPose2d.getX();
        //     double y = initialPose2d.getY();
        //     Rotation2d theta = initialPose2d.getRotation();
        //     theta = theta.rotateBy(Rotation2d.fromDegrees(180));
        //     initialPose2d = new Pose2d(new Translation2d(16.54 - x, y), theta);
        // }

        // final Pose2d finalInitialPose2d = initialPose2d;

        return Commands.sequence(
            Commands.runOnce(() -> swerveDrive.getImu().zeroAll()),
            autoBuilder.resetPose(path),
            autoBuilder.followPathWithEvents(path)
            // TODO: Once we get real odometry with vision, get rid of this
            // Commands.runOnce(() -> swerveDrive.setPoseMeters(finalInitialPose2d)),
            // autoCommand
        );
    }

    public static HashMap<String, Command> getEvents() {
        return events;
    }
}