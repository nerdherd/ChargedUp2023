package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BananaConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.Vision.PipelineType;


public class DriveToTarget extends CommandBase{
    private Limelight limelight;
    private PIDController pidX;
    private PIDController pidDistance;
    private SwerveDrivetrain drivetrain;
    private double goalArea;

    private static final double kMaxOutputPercent = 0.6;

    public DriveToTarget(SwerveDrivetrain drivetrain, Limelight limelight, double goalArea, PipelineType pipeline){
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.goalArea = goalArea;

        SmartDashboard.putNumber("tX P", 0.05);
        SmartDashboard.putNumber("area P", 0.24);
        SmartDashboard.putNumber("tX I", 0);
        SmartDashboard.putNumber("area I", 0);
        SmartDashboard.putNumber("tX D", 0);
        SmartDashboard.putNumber("area D", 0);

        // Replace PID values with working numbers later (PID values shouldn't affect anything since the robot gets the numbers from the SmartDashboard)
        switch(pipeline) {
            case CONE:
                limelight.setPipeline(1);
                pidX = new PIDController(0, 0, 0);
                pidDistance = new PIDController(0, 0, 0);
                break;
            case CUBE:
                limelight.setPipeline(2);
                pidX = new PIDController(0, 0, 0);
                pidDistance = new PIDController(0, 0, 0);
                break;
            case TAPE:
                limelight.setPipeline(3);
                pidX = new PIDController(0, 0, 0);
                pidDistance = new PIDController(0, 0, 0);
                break;
            case ATAG:
                limelight.setPipeline(4);
                pidX = new PIDController(0, 0, 0);
                pidDistance = new PIDController(0, 0, 0);
                break;
        }
        
        // Allows for tuning in Dashboard; Get rid of later once everything is tuned
        pidX = new PIDController(SmartDashboard.getNumber("tX P", 0.05), SmartDashboard.getNumber("tX I", 0.05), SmartDashboard.getNumber("tX D", 0.05)); //0.03
        pidX.setTolerance(0.2);
        pidDistance = new PIDController(SmartDashboard.getNumber("area P", 0.05), SmartDashboard.getNumber("area I", 0.05), SmartDashboard.getNumber("area D", 0.05)); //0.05
        pidDistance.setTolerance(0.2);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // double range = 0.628 - 1.71*Math.log(limelight.getArea());
        double objArea = limelight.getArea();
        double ySpeed = -pidX.calculate(limelight.getXAngle(), 0);   // SOMEBODY SWAP THE PIDX and Y NAMES
        double xSpeed = pidDistance.calculate(objArea, goalArea);
        
        ChassisSpeeds chassisSpeeds;
        
        xSpeed*=SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond; //*6
        ySpeed*=SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond; //*2

        SmartDashboard.putNumber("Vision X speed", xSpeed);
        SmartDashboard.putNumber("Vision Y speed", ySpeed);
        SmartDashboard.putNumber("Vision Area", objArea);
        SmartDashboard.putBoolean("Vision has target", limelight.hasValidTarget());
        SmartDashboard.putNumber("Limelight x", limelight.getXAngle());

        SmartDashboard.putBoolean("Setpoint reached x", pidX.atSetpoint());
        SmartDashboard.putBoolean("Setpoint reached y", pidDistance.atSetpoint());

        if(!limelight.hasValidTarget()) {
            xSpeed = 0;
            ySpeed = 0;
        }
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
        SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        drivetrain.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return pidX.atSetpoint() && pidDistance.atSetpoint();
    }

    public ChassisSpeeds getChassisSpeeds() {
        // double range = 0.628 - 1.71*Math.log(limelight.getArea());
        double objArea = limelight.getArea();
        double ySpeed = -pidX.calculate(limelight.getXAngle(), 0);   // SOMEBODY SWAP THE PIDX and Y NAMES
        double xSpeed = pidDistance.calculate(objArea, goalArea);

        ChassisSpeeds chassisSpeeds;

        xSpeed*=SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond; //*6
        ySpeed*=SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond; //*2

        SmartDashboard.putNumber("Vision X speed", xSpeed);
        SmartDashboard.putNumber("Vision Y speed", ySpeed);
        SmartDashboard.putNumber("Vision Area", objArea);
        SmartDashboard.putBoolean("Vision has target", limelight.hasValidTarget());
        SmartDashboard.putNumber("Limelight x", limelight.getXAngle());

        SmartDashboard.putBoolean("Setpoint reached x", pidX.atSetpoint());
        SmartDashboard.putBoolean("Setpoint reached y", pidDistance.atSetpoint());

        if(!limelight.hasValidTarget()) {
            xSpeed = 0;
            ySpeed = 0;
        }
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);

        return chassisSpeeds;
    }
}
