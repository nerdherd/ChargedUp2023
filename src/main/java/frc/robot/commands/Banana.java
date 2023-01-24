package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BananaConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.util.NerdyMath;

public class Banana extends SequentialCommandGroup {
    private Limelight limelight;
    private SwerveDrivetrain drivetrain;
    private PIDController pidController;

    public Banana(Limelight limelight, SwerveDrivetrain drivetrain){
        this.limelight = limelight;
        this.drivetrain = drivetrain;

        pidController = new PIDController(BananaConstants.kPIDControllerP, 0, BananaConstants.kPIDControllerD);
        pidController.setTolerance(0.5);

        limelight.setPipeline(4);

        addCommands(
            new TurnToAngle(180, drivetrain),
            new InstantCommand(() -> driveToTarget())
        );
    }

    private void driveToTarget(){
        double range = 0.628 - 1.71*Math.log(getTA());
        double pidTX = pidController.calculate(getTX(), 0);
        double pidRange = pidController.calculate(range, VisionConstants.kGoalRangeMeters);
        ChassisSpeeds chassisSpeeds;
        if(pidController.atSetpoint()){
            chassisSpeeds = new ChassisSpeeds(0, 0, 0);
        }
        else{
            chassisSpeeds = new ChassisSpeeds(pidTX, pidRange, 0);
        }
        SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        drivetrain.setModuleStates(moduleStates);
    }

    private double getTX(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }
    private double getTA(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    }
    private double getTY(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    }
}
