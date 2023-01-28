package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrivetrain;

public class SwerveDriveBy extends CommandBase {
    private PIDController xPidController = new PIDController(SwerveAutoConstants.kPXController, 0, 0);
    private PIDController yPidController = new PIDController(SwerveAutoConstants.kPXController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        SwerveAutoConstants.kPThetaController, 0, 0, 
        SwerveAutoConstants.kThetaControllerConstraints);
    
    private SwerveDrivetrain swerveDrive;
    private Pose2d targetPose;

    /**
     * Command for the swerve drive to drive xDistance and yDistance
     */
    public SwerveDriveBy(SwerveDrivetrain swerveDrive, double xDistance, double yDistance, double angle) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
        xPidController.setTolerance(0.1);
        yPidController.setTolerance(0.1);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        Pose2d currentPose = swerveDrive.getPose();
        targetPose = new Pose2d(
            currentPose.getX() + xDistance, 
            currentPose.getY() + yDistance, 
            Rotation2d.fromDegrees(angle));
    }

    @Override
    public void initialize() {
        thetaController.setGoal(targetPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerveDrive.getPose();
        double xSpeed = xPidController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yPidController.calculate(currentPose.getX(), targetPose.getX());
        double thetaSpeed = thetaController.calculate(currentPose.getRotation().getRadians());

        ChassisSpeeds chassisSpeeds = 
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, thetaSpeed, swerveDrive.getRotation2d());
        
        SwerveModuleState[] swerveModuleStates = 
            SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        
        swerveDrive.setModuleStates(swerveModuleStates);
    }

    @Override
    public boolean isFinished() {
        return (xPidController.atSetpoint() && yPidController.atSetpoint());
    }
}
