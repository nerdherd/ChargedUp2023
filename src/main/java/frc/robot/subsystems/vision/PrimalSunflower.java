package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.claw.MotorClaw;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.Limelight.LightMode;

public class PrimalSunflower {
    private Double[][] gridPositions = {
        {},
        {},
        {},
        {},
        {},
        {},
        {},
        {},
        {}
    };

    private Limelight limelight;

    private Arm arm;
    private Elevator elevator;
    private MotorClaw claw;
    private SwerveDrivetrain drivetrain;

    private PIDController PIDArea = new PIDController(0, 0, 0);
    private PIDController PIDTX = new PIDController(0, 0, 0);
    private PIDController PIDYaw = new PIDController(0, 0, 0);

    private String llname;

    public PrimalSunflower(String limelightName, Arm arm, Elevator elevator, MotorClaw claw, SwerveDrivetrain drivetrain) {
        this.arm = arm;
        this.elevator = elevator;
        this.claw = claw;
        this.drivetrain = drivetrain;
        this.llname = limelightName;

        try {
            limelight = new Limelight(limelightName); //TODO: Change to actual limelight name
            limelight.setLightState(LightMode.OFF);
        } catch (Exception e) {
            limelight = null;
            DriverStation.reportWarning("Error instantiating limelight with name " + limelightName + ": " + e.getMessage(), true);
        }

        SmartDashboard.putNumber("Tx P", 0);       
        SmartDashboard.putNumber("Tx I", 0);
        SmartDashboard.putNumber("Tx D", 0);

        SmartDashboard.putNumber("Ta P", 0);       
        SmartDashboard.putNumber("Ta I", 0);
        SmartDashboard.putNumber("Ta D", 0);

        SmartDashboard.putNumber("Yaw P", 0);       
        SmartDashboard.putNumber("Yaw I", 0);
        SmartDashboard.putNumber("Yaw D", 0);
    }
    

    private Double[] generateSun() {
        Double[] yee = {0.0, 0.0, 0.0};
        if(limelight == null) return yee;
        Pose3d pos = new Pose3d();
        double x = 0.0, y = 0.0, z = 0.0;
        if(limelight.hasValidTarget()) {
            pos = LimelightHelpers.getBotPose3d(llname); // Replace w different met.
            return new Double[]{pos.getX(), pos.getY(), pos.getZ()};

        }
        return yee;
    }

    public Double[] getClosestZombie() {
        Double[] robotPos = generateSun();
        int gridNumber = 0;
        Double distance = Math.sqrt(Math.pow(gridPositions[0][1] - robotPos[1], 2) + Math.pow(gridPositions[0][0] - robotPos[0], 2)); // distance formula
        for (int i = 0; i < gridPositions.length; i++) {
            Double newDistance = Math.sqrt(Math.pow(gridPositions[i][1] - robotPos[1], 2) + Math.pow(gridPositions[i][0] - robotPos[0], 2)); // distance formula
            if(newDistance < distance) {
                distance = newDistance;
                gridNumber = i;
            }
        }
        SmartDashboard.putNumber("Closest Grid:", gridNumber);
        return gridPositions[gridNumber];
    }
}
