package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrivetrain;

public class TheGreatBalancingAct extends CommandBase {
    private SwerveDrivetrain swerveDrive;
    private PIDController rollPidController;
    private PIDController pitchPidController;

    // private double towTime = 0;
    // private double period = 0.02;

    /**
     * Construct a new TurnToAngle Command
     * @param swerveDrive   Swerve drivetrain to rotate
     * @param period      
     *   Time between each calculation (default 20ms)
     */
    public TheGreatBalancingAct(SwerveDrivetrain swerveDrive, double period, double pBalancing) {
        this.swerveDrive = swerveDrive;

        this.rollPidController = new PIDController(
            pBalancing, 0, 0, period);
        
        this.rollPidController.enableContinuousInput(0, 360);
        
        this.pitchPidController = new PIDController(
            pBalancing, 0, 0, period);
        
        this.pitchPidController.enableContinuousInput(0, 360);
        addRequirements(swerveDrive);

        // this.period = period;
    }

    public TheGreatBalancingAct(SwerveDrivetrain swerveDrive, double pBalancing) {
        this(swerveDrive, 0.02, pBalancing);
    }

    public TheGreatBalancingAct(SwerveDrivetrain swerveDrive) {
        // Default period is 20 ms
        this(swerveDrive, 0.02, SwerveAutoConstants.kPBalancing);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Rotation3d currentRotation = swerveDrive.getRotation3d();

        // Calculate turning speed with PID
        double xSpeed = -7.5 * pitchPidController.calculate(
            currentRotation.getY(), 0
        );
        double ySpeed = -7.5 * rollPidController.calculate(
            currentRotation.getX(), 0
        );

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("pitch", currentRotation.getY());
        SmartDashboard.putNumber("roll", currentRotation.getX());
        SmartDashboard.putNumber("yaw", currentRotation.getZ());

        // Tow the swerve
        // if (currentRotation.getX() < SwerveAutoConstants.kBalancingDeadbandDegrees
        //     && currentRotation.getY() < SwerveAutoConstants.kBalancingDeadbandDegrees) {
        //     towTime += period;
        // } else {
        //     towTime = 0;
        // }

        // if (towTime > SwerveAutoConstants.kBalancingTowPeriod) {
        //     swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates);
        //     return;
        // }

        // Convert speed into swerve states
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0.0);
        SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        
        // Set swerve states
        swerveDrive.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
