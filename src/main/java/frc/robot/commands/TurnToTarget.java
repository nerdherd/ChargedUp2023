package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BananaConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision.PipelineType;


public class TurnToTarget extends CommandBase{
    private Limelight limelight;
    private PIDController pidAngle;
    private PIDController pidDistance;
    private SwerveDrivetrain drivetrain;
    private double goalArea;

    private static final double kMaxOutputPercent = 0.6;

    public TurnToTarget(SwerveDrivetrain drivetrain, Limelight limelight, double goalArea, PIDController pidAngle, PIDController pidDistance){
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.goalArea = goalArea;

        SmartDashboard.putNumber("tX P", 0.05);
        SmartDashboard.putNumber("area P", 0.24);
        SmartDashboard.putNumber("tX I", 0);
        SmartDashboard.putNumber("area I", 0);
        SmartDashboard.putNumber("tX D", 0);
        SmartDashboard.putNumber("area D", 0);
        
        // Allows for tuning in Dashboard; Get rid of later once everything is tuned
        this.pidAngle = pidAngle;
        pidAngle.setTolerance(0.2);
        this.pidDistance = pidDistance;
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
        double angularSpeed = -pidAngle.calculate(limelight.getXAngle(), 0);   // SOMEBODY SWAP THE PIDX and Y NAMES
        double distanceSpeed = pidDistance.calculate(objArea, goalArea);
        
        ChassisSpeeds chassisSpeeds;
        
        distanceSpeed*=SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond; //*6
        angularSpeed*=SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond; //*2

        SmartDashboard.putNumber("Vision X speed", distanceSpeed);
        SmartDashboard.putNumber("Vision Y speed", angularSpeed);
        SmartDashboard.putNumber("Vision Area", objArea);
        SmartDashboard.putBoolean("Vision has target", limelight.hasValidTarget());
        SmartDashboard.putNumber("Limelight x", limelight.getXAngle());

        SmartDashboard.putBoolean("Setpoint reached x", pidAngle.atSetpoint());
        SmartDashboard.putBoolean("Setpoint reached y", pidDistance.atSetpoint());

        if(!limelight.hasValidTarget()) {
            distanceSpeed = 0;
            angularSpeed = 0;
        }
        chassisSpeeds = new ChassisSpeeds(distanceSpeed, 0, angularSpeed);
        SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        drivetrain.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return pidAngle.atSetpoint() && pidDistance.atSetpoint();
    }

    public ChassisSpeeds getChassisSpeeds() {
        // double range = 0.628 - 1.71*Math.log(limelight.getArea());
        double objArea = limelight.getArea();
        double angularSpeed = -pidAngle.calculate(limelight.getXAngle(), 0);   // SOMEBODY SWAP THE PIDX and Y NAMES
        double distanceSpeed = pidDistance.calculate(objArea, goalArea);

        ChassisSpeeds chassisSpeeds;

        distanceSpeed*=SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond; //*6
        angularSpeed*=SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond; //*2

        SmartDashboard.putNumber("Vision X speed", distanceSpeed);
        SmartDashboard.putNumber("Vision Y speed", angularSpeed);
        SmartDashboard.putNumber("Vision Area", objArea);
        SmartDashboard.putBoolean("Vision has target", limelight.hasValidTarget());
        SmartDashboard.putNumber("Limelight x", limelight.getXAngle());

        SmartDashboard.putBoolean("Setpoint reached x", pidAngle.atSetpoint());
        SmartDashboard.putBoolean("Setpoint reached y", pidDistance.atSetpoint());

        if(!limelight.hasValidTarget()) {
            distanceSpeed = 0;
            angularSpeed = 0;
        }
        chassisSpeeds = new ChassisSpeeds(distanceSpeed, 0, angularSpeed);

        return chassisSpeeds;
    }
}