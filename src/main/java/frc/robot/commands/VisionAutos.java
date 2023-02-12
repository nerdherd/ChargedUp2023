package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision.PipelineType;

public class VisionAutos {

    public static CommandBase penPineappleApplePen(SwerveDrivetrain drivetrain, Limelight limelight) {

        SmartDashboard.putBoolean("called 1", true);
        return new RunCommand(() -> getPPAP(drivetrain, limelight));
        // return new DriveToTarget(drivetrain, limelight, 2, PipelineType.CONE);
    }

    public static void getPPAP(SwerveDrivetrain drivetrain, Limelight limelight) {
        SmartDashboard.putNumber("tX P", 0.05);
        SmartDashboard.putNumber("area P", 0.24);
        SmartDashboard.putNumber("tX I", 0);
        SmartDashboard.putNumber("area I", 0);
        SmartDashboard.putNumber("tX D", 0);
        SmartDashboard.putNumber("area D", 0);

        limelight.setPipeline(1);

        PIDController pidX;
        PIDController pidDistance;

        // Allows for tuning in Dashboard; Get rid of later once everything is tuned
        pidX = new PIDController(SmartDashboard.getNumber("tX P", 0.05), SmartDashboard.getNumber("tX I", 0.05), SmartDashboard.getNumber("tX D", 0.05)); //0.03
        double tolerance = 0.1 * limelight.getArea();
        pidX.setTolerance(0.2);
        pidDistance = new PIDController(SmartDashboard.getNumber("area P", 0.05), SmartDashboard.getNumber("area I", 0.05), SmartDashboard.getNumber("area D", 0.05)); //0.05
        pidDistance.setTolerance(tolerance);
        pidX.setTolerance(tolerance);
        double goalArea = 10;

        final double kMaxOutputPercent = 0.6;
        

        // double range = 0.628 - 1.71*Math.log(limelight.getArea());
        double objArea = limelight.getArea();
        
        ChassisSpeeds chassisSpeeds;
        
        double calculatedX, calculatedY;

        double xSpeed;
        double ySpeed;

        calculatedX = pidDistance.calculate(objArea, goalArea);
        calculatedY = -pidX.calculate(limelight.getXAngle(), 0);
        if(pidDistance.atSetpoint()) {
            xSpeed = 0;
        }
        else {
            xSpeed = calculatedX;
            xSpeed*=SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond; //*6
        }

        if(pidX.atSetpoint()){
            ySpeed = 0;
        }
        else {
            ySpeed = calculatedY;   // SOMEBODY SWAP THE PIDX and Y NAMES
            ySpeed*=SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond; //*2
        }

        SmartDashboard.putNumber("Vision Tolerance", tolerance);
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
        // drivetrain.setModuleStates(moduleStates);
    }
    
}
