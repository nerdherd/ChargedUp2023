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

public class DriveToTarget extends CommandBase{
    private Limelight limelight;
    private PIDController pidX;
    private PIDController pidY;
    private SwerveDrivetrain drivetrain;
    private double goalArea;

    private static final double kMaxOutputPercent = 0.6;

    public static enum pipeline {
        CONE,
        CUBE,
        TAPE,
        ATAG
    }

    public DriveToTarget(SwerveDrivetrain drivetrain, Limelight limelight, double goalArea, pipeline pipeline){
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.goalArea = goalArea;

        switch(pipeline) {
            case CONE:
                limelight.setPipeline(1);
                break;
            case CUBE:
                limelight.setPipeline(2);
                break;
            case TAPE:
                limelight.setPipeline(3);
                break;
            case ATAG:
                limelight.setPipeline(4);
                break;
        }
        
        pidX = new PIDController(BananaConstants.kPIDControllerP, 0, BananaConstants.kPIDControllerD);
        pidX.setTolerance(0.5);
        pidY = new PIDController(BananaConstants.kPIDControllerP, 0, BananaConstants.kPIDControllerD);
        pidY.setTolerance(0.5);

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
        double xSpeed = -pidY.calculate(objArea, goalArea);

        ChassisSpeeds chassisSpeeds;


        SmartDashboard.putNumber("Vision X speed", xSpeed);
        SmartDashboard.putNumber("Vision Y speed", ySpeed);
        SmartDashboard.putNumber("Vision Area", objArea);
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

        double objArea = limelight.getArea();
        double ySpeed = pidX.calculate(limelight.getXAngle(), 0);
        double xSpeed = pidY.calculate(objArea, goalArea);

        ChassisSpeeds chassisSpeeds;


        SmartDashboard.putNumber("Vision X speed", xSpeed);
        SmartDashboard.putNumber("Vision Y speed", ySpeed);
        SmartDashboard.putNumber("Vision Area", objArea);
        SmartDashboard.putBoolean("Vision has target", limelight.hasValidTarget());
        SmartDashboard.putNumber("Limelight x", limelight.getXAngle());

        SmartDashboard.putBoolean("Setpoint reached x", pidX.atSetpoint());
        SmartDashboard.putBoolean("Setpoint reached y", pidY.atSetpoint());
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
        
        //SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        return chassisSpeeds;
    }
}
