package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BananaConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.util.NerdyMath;

public class DriveToTarget extends CommandBase{
    private Limelight limelight;
    private PIDController pidX;
    private PIDController pidY;
    private SwerveDrivetrain drivetrain;
    private double distance;

    private static final double kMaxOutputPercent = 0.6;

    public DriveToTarget(Limelight limelight, SwerveDrivetrain drivetrain, double distanceInMeters){
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        distance = distanceInMeters;
        
        pidX = new PIDController(BananaConstants.kPIDControllerP, 0, BananaConstants.kPIDControllerD);
        pidX.setTolerance(0.5);
        pidY = new PIDController(BananaConstants.kPIDControllerP, 0, BananaConstants.kPIDControllerD);
        pidY.setTolerance(0.5);

        limelight.setPipeline(0);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // double range = 0.628 - 1.71*Math.log(limelight.getArea());
        double range = limelight.getArea();
        double xSpeed = pidX.calculate(limelight.getXAngle(), 0);
        double ySpeed = pidY.calculate(range, distance);

        xSpeed = NerdyMath.clamp(xSpeed, -kMaxOutputPercent, kMaxOutputPercent);
        ySpeed = NerdyMath.clamp(ySpeed, -kMaxOutputPercent, kMaxOutputPercent);

        ChassisSpeeds chassisSpeeds;
        if(pidX.atSetpoint()){
            xSpeed = 0;
        }
        if(pidY.atSetpoint()){
            ySpeed = 0;
        }

        SmartDashboard.putNumber("Vision X speed", xSpeed);
        SmartDashboard.putNumber("Vision Y speed", ySpeed);
        SmartDashboard.putNumber("Vision Range", range);
        SmartDashboard.putBoolean("Vision has target", limelight.hasValidTarget());
        SmartDashboard.putNumber("Limelight x", limelight.getXAngle());

        SmartDashboard.putBoolean("Setpoint reached x", pidX.atSetpoint());
        SmartDashboard.putBoolean("Setpoint reached y", pidY.atSetpoint());
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
        SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        drivetrain.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return pidX.atSetpoint() && pidY.atSetpoint();
    }

    public ChassisSpeeds getChassisSpeeds() {
        double range = 0.628 - 1.71*Math.log(limelight.getArea());
        SmartDashboard.putNumber("Range: ", range);
        double xSpeed = pidX.calculate(limelight.getXAngle(), 0);
        double ySpeed = pidY.calculate(range, distance);

        xSpeed = NerdyMath.clamp(xSpeed, -kMaxOutputPercent, kMaxOutputPercent);
        ySpeed = NerdyMath.clamp(ySpeed, -kMaxOutputPercent, kMaxOutputPercent);

        if(pidX.atSetpoint()){
            xSpeed = 0;
        }
        if(pidY.atSetpoint()){
            ySpeed = 0;
        }

        return new ChassisSpeeds(xSpeed, ySpeed, 0);
    }
}
