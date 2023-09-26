package frc.robot.subsystems.vision.primalWallnut;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Reportable;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.Limelight.LightMode;
import frc.robot.subsystems.vision.jurrasicMarsh.LimelightHelperUser;
import frc.robot.subsystems.vision.jurrasicMarsh.LimelightHelpers;

public class PrimalSunflower implements Reportable {
    // XYZ coordinates of all grid positions.
    // Relative to bottom left corner of field (closest to blue alliance cable side) being (0, 0, 0)
    private Double[][] gridPositions = {
        // {6.5, 1.1, 0.0},
        // {6.5, 0.4, 0.0},
        // {6.5, -0.15, 0.0},
        // {6.5, -0.7, 0.0},
        // {6.5, -1.25, 0.0},
        // {6.5, -1.8, 0.0},
        // {6.5, -2.35, 0.0},
        // {6.5, -2.95, 0.0},
        // {6.5, -3.55, 0.0}

        {1.75, 0.41, 0.0},
        {1.75, 1.06, 0.0},
        {1.75, 1.61, 0.0},
        {1.75, 2.19, 0.0},
        {1.75, 2.75, 0.0},
        {1.75, 3.30, 0.0},
        {1.75, 3.87, 0.0},
        {1.75, 4.42, 0.0},
        {1.75, 5.06, 0.0}
    };

    //robot position
    private Double[] robotPos = {0.0, 0.0, 0.0};

    //points in the path to get to the closest grid
    PathPoint firstPoint = new PathPoint(new Translation2d(robotPos[0], robotPos[1]), Rotation2d.fromDegrees(0));
    PathPoint secondPoint = new PathPoint(new Translation2d(robotPos[0], robotPos[1]), Rotation2d.fromDegrees(0));
    PathPoint thirdPoint = new PathPoint(new Translation2d(robotPos[0], robotPos[1]), Rotation2d.fromDegrees(0));

    private Limelight limelight;
    private LimelightHelperUser limelightUser;

    private PIDController PIDArea = new PIDController(0, 0, 0);
    private PIDController PIDTX = new PIDController(0, 0, 0);
    private PIDController PIDYaw = new PIDController(0, 0, 0);

    private String llname;

    private Field2d field;

    /*
     * Params:
     * limelightName = name of the limelight
     * drivetrain = swerve drive 
    */
    public PrimalSunflower(String limelightName) {
        // this.drivetrain = drivetrain;   UNCOMMENR LATER
        try {
            SmartDashboard.putBoolean("LimelightHelper inited", true);
            limelight = new Limelight(limelightName);
            limelight.setLightState(LightMode.OFF);
            limelight.setPipeline(4);
            limelightUser = new LimelightHelperUser(limelightName);
            
        } catch (Exception e) {
            limelight = null;
            // DriverStation.reportWarning("Error instantiating limelight with name " + limelightName + ": " + e.getMessage(), true);
            SmartDashboard.putBoolean("LimelightHelper inited", false);
            limelightUser = null;
        }

        field = new Field2d();

        // SmartDashboard.putNumber("Tx P", 0);       
        // SmartDashboard.putNumber("Tx I", 0);
        // SmartDashboard.putNumber("Tx D", 0);

        // SmartDashboard.putNumber("Ta P", 0);       
        // SmartDashboard.putNumber("Ta I", 0);
        // SmartDashboard.putNumber("Ta D", 0);

        // SmartDashboard.putNumber("Yaw P", 0);       
        // SmartDashboard.putNumber("Yaw I", 0);
        // SmartDashboard.putNumber("Yaw D", 0);
    }
    

    //get robot position if limelight has target else, return 0, 0, 0 (https://docs.limelightvision.io/en/latest/coordinate_systems_fiducials.html#field-space)
    public Double[] generateSun() {
        Double[] yee = {0.0, 0.0, 0.0};
        if (limelight == null) {
            return yee;
        }
        limelight.setPipeline(VisionConstants.kAprilTagPipeline);
        
        Pose3d pos = new Pose3d();
        if(limelight.hasValidTarget()) {
            pos = limelightUser.getPose3d(); // Replace w different met.
            field.setRobotPose(pos.toPose2d());
            return new Double[]{pos.getX(), pos.getY(), pos.getZ()};
        }
        return yee;
    }

    public Pose3d getPose3d() {
        if (limelight == null) {
            return null;
        }

        limelight.setPipeline(VisionConstants.kAprilTagPipeline);
        
        if(limelight.hasValidTarget()) {
            return limelightUser.getPose3d(); // Replace w different met? Idk i just copied it from generateSun()
        } 

        return null;
    }

    /**
     * @return index of the closest grid to the robot
     */
    public int getClosestZombieLane() {
        robotPos = generateSun();
        int gridNumber = 0;
        Double distance = Math.sqrt(Math.pow(gridPositions[0][1] - robotPos[1], 2) + Math.pow(gridPositions[0][0] - robotPos[0], 2)); // distance formula
        for (int i = 0; i < gridPositions.length; i++) {
            Double newDistance = Math.sqrt(Math.pow(gridPositions[i][1] - robotPos[1], 2) + Math.pow(gridPositions[i][0] - robotPos[0], 2)); // distance formula
            if(newDistance < distance) {
                distance = newDistance;
                gridNumber = i;
            }
        }

        return gridNumber;
    }

    /**
     * @return position of the closest grid to the robot
     */
    public Double[] getClosestZombieTile() {
        return gridPositions[getClosestZombieLane()];
    }

    /**
     * @return PathPlannerTrajectory to get to the closest grid
     */
    public PathPlannerTrajectory toNearestGrid(SwerveDrivetrain swerveDrive) {
        robotPos = generateSun();
        Double[] gridPos = getClosestZombieTile();

        return PathPlanner.generatePath(
            PathPlannerConstants.kPPPathConstraints,
            List.of(
                new PathPoint(swerveDrive.getPose().getTranslation(), swerveDrive.getPose().getRotation()),
                new PathPoint(new Translation2d(gridPos[0], gridPos[1]), Rotation2d.fromDegrees(180))
            )
        );
    }

    /**
     * Debug method to generate trajectory to nearest grid and display on shuffleboard.
     * 
     * @return Trajectory to get to the closest grid.
     */
    public Trajectory toNearestGridDebug(SwerveDrivetrain swerveDrive) {
        robotPos = generateSun();
        Double[] gridPos = getClosestZombieTile(); // Get coordinates of closest grid

        Trajectory trajectory = 
            TrajectoryGenerator.generateTrajectory(
                List.of(
                    swerveDrive.getPose(),
                    new Pose2d(new Translation2d(gridPos[0], gridPos[1]), Rotation2d.fromDegrees(180))
                ),
                new TrajectoryConfig(PathPlannerConstants.kPPMaxVelocity, PathPlannerConstants.kPPMaxAcceleration) // constants for debugging purposes
            );

        field.getObject("traj").setTrajectory(trajectory);
        return trajectory; 
    }

    public void reportToSmartDashboard(LOG_LEVEL priority) {
        if(limelightUser != null) limelightUser.reportToSmartDashboard();
    }

    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF)  {
            return;
        }
        ShuffleboardTab tab;

        switch (level) {
            case OFF:
                break;
            case ALL:
                tab = Shuffleboard.getTab("Vision");
                tab.addNumber("Robot Pose X", () -> generateSun()[0]);
                tab.addNumber("Robot Pose Y", () -> generateSun()[1]);
                tab.addNumber("Robot Pose Z", () -> generateSun()[2]);

                tab.addNumber("Closest Grid X", () -> getClosestZombieTile()[0]);
                tab.addNumber("Closest Grid Y", () -> getClosestZombieTile()[1]);
                tab.addNumber("Closest Grid Z", () -> getClosestZombieTile()[2]);

                tab.addNumber("Closest Grid ID", () -> getClosestZombieLane());
                tab.addBoolean("AprilTag Found", () -> limelight.hasValidTarget());
                
                // Only trajectory point is the grid position now.
                // tab.addNumber("Traj Point 1 Pose X", () -> firstPoint.position.getX());
                // tab.addNumber("Traj Point 1 Pose Y", () -> firstPoint.position.getY());

                // tab.addNumber("Traj Point 2 Pose X", () -> secondPoint.position.getX());
                // tab.addNumber("Traj Point 2 Pose Y", () -> secondPoint.position.getY());

                // tab.addNumber("Traj Point 3 Pose X", () -> thirdPoint.position.getX());
                // tab.addNumber("Traj Point 3 Pose Y", () -> thirdPoint.position.getY());

                tab.add("Field Position", field).withSize(6, 3);
            case MEDIUM:
                
            case MINIMAL:
                
                break;
        }
    }
}
