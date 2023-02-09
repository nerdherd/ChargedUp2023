package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Vision.PipelineType;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class ApproachCombined extends CommandBase {
    
    SwerveDrivetrain drivetrain;
    DriveToTarget driveToTarget;
    TurnToAngle turnToAngle;

    /**
     * Move swerve drivetrain to approach vision target.
     * 
     * @param meterOffset   Setpoint distance between drivetrain and vision target
     */
    public ApproachCombined(SwerveDrivetrain drivetrain, double orientation, double meterOffset, Limelight limelight, PIDController pidX, PIDController pidDistance) {
        this.drivetrain = drivetrain;
        
        driveToTarget = new DriveToTarget(drivetrain, limelight, meterOffset, pidX, pidDistance);
        turnToAngle = new TurnToAngle(orientation, drivetrain);
    }

    @Override
    public void execute() {
        ChassisSpeeds finalChassisSpeeds;
        ChassisSpeeds translationSpeeds = driveToTarget.getChassisSpeeds();
        ChassisSpeeds rotationSpeeds = turnToAngle.getChassisSpeeds();

        // Combine speeds directly without a weight,
        // since DriveToTarget only affects x and y speeds, while TurnToAngle only affects rotation speed
        finalChassisSpeeds = new ChassisSpeeds(translationSpeeds.vxMetersPerSecond, 
            translationSpeeds.vyMetersPerSecond, 
            rotationSpeeds.omegaRadiansPerSecond);

        SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(finalChassisSpeeds);
        drivetrain.setModuleStates(moduleStates);
    }

    
    @Override
    public boolean isFinished() {
        return driveToTarget.isFinished() && turnToAngle.isFinished();
    }

    // Stop the drivetrain from moving when command ends
    // Do I need this or will it stop on its own?
    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds zero = new ChassisSpeeds(
            0, 0, 0);

        SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(zero);
        drivetrain.setModuleStates(moduleStates);
    }

}
