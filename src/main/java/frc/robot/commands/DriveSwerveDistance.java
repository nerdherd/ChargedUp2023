package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.subsystems.SwerveDrivetrain;

public class DriveSwerveDistance extends CommandBase {
    private PIDController pidController = new PIDController(SwerveAutoConstants.kPXController, 0, 0);

    private SwerveDrivetrain swerveDrive;
    private double targetDistance;
    private double angle;
    private double initialPose;
    private double endPose;

    /**
     * Doesnt work yet
     */
    public DriveSwerveDistance(SwerveDrivetrain swerveDrive, double distance, double angle) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
        pidController.setTolerance(0.1);

        targetDistance = distance;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        initialPose = swerveDrive.getModulePositions()[0].distanceMeters;
        endPose = initialPose + targetDistance;
    }

    @Override
    public void execute() {
        double distanceDifference = endPose - swerveDrive.getModulePositions()[0].distanceMeters;

        SwerveModuleState[] moduleStates = {
            new SwerveModuleState(pidController.calculate(distanceDifference), Rotation2d.fromDegrees(angle)),
            new SwerveModuleState(pidController.calculate(distanceDifference), Rotation2d.fromDegrees(angle)),
            new SwerveModuleState(pidController.calculate(distanceDifference), Rotation2d.fromDegrees(angle)),
            new SwerveModuleState(pidController.calculate(distanceDifference), Rotation2d.fromDegrees(angle))
        };

        swerveDrive.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        return (pidController.atSetpoint());
    }
}
