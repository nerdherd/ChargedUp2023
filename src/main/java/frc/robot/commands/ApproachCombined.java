package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;

public class ApproachCombined extends CommandBase {
    
    SwerveDrivetrain drivetrain;
    DriveToTarget driveToTarget;
    TurnToAngle turnToAngle;

    /**
     * Move swerve drivetrain to approach vision target.
     * 
     * @param meterOffset   Setpoint distance between drivetrain and vision target
     */
    public ApproachCombined(Limelight limelight, SwerveDrivetrain drivetrain, double meterOffset) {
        this.drivetrain = drivetrain;
        
        driveToTarget = new DriveToTarget(limelight, drivetrain, meterOffset);
        turnToAngle = new TurnToAngle(0, drivetrain);
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
