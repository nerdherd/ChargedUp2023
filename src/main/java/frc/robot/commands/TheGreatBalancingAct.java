package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrivetrain;

public class TheGreatBalancingAct extends CommandBase {
    private SwerveDrivetrain swerveDrive;
    private PIDController rollPidController;
    private PIDController pitchPidController;

    /**
     * Construct a new TurnToAngle Command
     * @param swerveDrive   Swerve drivetrain to rotate
     * @param period        Time between each calculation (default 20ms)
     */
    public TheGreatBalancingAct(SwerveDrivetrain swerveDrive, double period) {
        this.swerveDrive = swerveDrive;

        this.rollPidController = new PIDController(
            AutoConstants.kPBalancing, 0, 0, period);
        
        this.rollPidController.enableContinuousInput(0, 360);
        
        this.pitchPidController = new PIDController(
            AutoConstants.kPBalancing, 0, 0, period);
        
        this.pitchPidController.enableContinuousInput(0, 360);
        addRequirements(swerveDrive);
    }

    public TheGreatBalancingAct(SwerveDrivetrain swerveDrive) {
        // Default period is 20 ms
        this(swerveDrive, 0.02);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Rotation3d currentRotation = swerveDrive.getRotation3d();

        // Calculate turning speed with PID
        double xSpeed = pitchPidController.calculate(
            currentRotation.getY(), 0
        );
        double ySpeed = rollPidController.calculate(
            currentRotation.getX(), 0
        );

        // Convert speed into swerve states
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, 0, swerveDrive.getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        
        // Set swerve states
        swerveDrive.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
