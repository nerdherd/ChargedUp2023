package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrivetrain;

public class TurnToAngle extends CommandBase {
    private double targetAngle;
    private SwerveDrivetrain swerveDrive;
    private PIDController pidController;

    /**
     * Construct a new TurnToAngle Command
     * @param targetAngle   Target angle (degrees)
     * @param swerveDrive   Swerve drivetrain to rotate
     * @param period        Time between each calculation (default 20ms)
     */
    public TurnToAngle(double targetAngle, SwerveDrivetrain swerveDrive, double period) {
        this.targetAngle = targetAngle;
        this.swerveDrive = swerveDrive;

        this.pidController = new PIDController(
            AutoConstants.kPThetaController, 0, 0, period);
        
        this.pidController.setTolerance(
            AutoConstants.kTurnToAnglePositionToleranceAngle, 
            AutoConstants.kTurnToAngleVelocityToleranceAnglesPerSec * period);
        
        this.pidController.enableContinuousInput(0, 360);
        
        addRequirements(swerveDrive);
    }

    public TurnToAngle(double targetAngle, SwerveDrivetrain swerveDrive) {
        // Default period is 20 ms
        this(targetAngle, swerveDrive, 0.02);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Calculate turning speed with PID
        double turningSpeed = pidController.calculate(swerveDrive.getHeading(), targetAngle);
        
        // Convert speed into swerve states
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                0, 0, turningSpeed, swerveDrive.getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        
        // Set swerve states
        swerveDrive.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}
