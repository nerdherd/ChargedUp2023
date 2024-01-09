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

    private double taRequirement;

    /*
     * Params:
     * limelightName = name of the limelight
     * taRequirement = the minimum ta in order for pose estimator to include vision measurement
    */
    public PrimalSunflower(String limelightName, double taRequirement) {
        // this.drivetrain = drivetrain;   UNCOMMENR LATER
        this.taRequirement = taRequirement;
        llname = limelightName;
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

    public double getPose3dXCoord() {
        if (limelight == null) {
            return 0;
        }

        limelight.setPipeline(VisionConstants.kAprilTagPipeline);
        
        if(limelight.hasValidTarget()) {
            return limelightUser.getX(); // Replace w different met? Idk i just copied it from generateSun()
        } 

        return 0;
    }

    public double getPose3dYCoord() {
        if (limelight == null) {
            return 0;
        }

        limelight.setPipeline(VisionConstants.kAprilTagPipeline);
        
        if(limelight.hasValidTarget()) {
            return limelightUser.getY(); // Replace w different met? Idk i just copied it from generateSun()
        } 

        return 0;
    }

    /**
    * @return Area of apriltag from camera
    */
    public double getSunSize() {
        return limelight.getArea();
    }

    /**
     * 
     * @return Tag size requirement to be reliable enough to use
     */
    public double getOptimalSunSize() {
        return taRequirement;
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
                tab = Shuffleboard.getTab(llname);
                tab.addNumber("Robot Pose X", () -> generateSun()[0]);
                tab.addNumber("Robot Pose Y", () -> generateSun()[1]);
                tab.addNumber("Robot Pose Z", () -> generateSun()[2]);

                tab.addBoolean("AprilTag Found", () -> limelight.hasValidTarget());
                
                // Only trajectory point is the grid position now.
                // tab.addNumber("Traj Point 1 Pose X", () -> firstPoint.position.getX());
                // tab.addNumber("Traj Point 1 Pose Y", () -> firstPoint.position.getY());

                // tab.addNumber("Traj Point 2 Pose X", () -> secondPoint.position.getX());
                // tab.addNumber("Traj Point 2 Pose Y", () -> secondPoint.position.getY());

                // tab.addNumber("Traj Point 3 Pose X", () -> thirdPoint.position.getX());
                // tab.addNumber("Traj Point 3 Pose Y", () -> thirdPoint.position.getY());

                // tab.add("Field Position", field).withSize(6, 3);
            case MEDIUM:
                
            case MINIMAL:
                
                break;
        }
    }
}