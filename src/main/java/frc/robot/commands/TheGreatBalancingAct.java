package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class TheGreatBalancingAct extends CommandBase {
    private SwerveDrivetrain swerveDrive;
    private PIDController rollPidController;
    private PIDController pitchPidController;

    // private double towTime = 0;
    // private double period = 0.02;

    /**
     * Construct a new BalancingAct Command
     * @param swerveDrive   Swerve drivetrain to balance
     * @param period        Time intervals for pid controller calculations (default 20ms)
     * @param pBalancing    Constant P for roll and pitch pid controllers
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

    /**
     * Construct a new BalancingAct Command and assume period is the default (20ms)
     * @param swerveDrive   Swerve drivetrain to balance
     * @param pBalancing    Constant P for roll and pitch pid controllers
     */
    public TheGreatBalancingAct(SwerveDrivetrain swerveDrive, double pBalancing) {
        this(swerveDrive, 0.02, pBalancing);
    }

    /**
     * Construct a new BalancingAct Command
     * assumes period is the default (20ms)
     * Uses swerve auto constant for roll and pitch pid controllers
     * @param swerveDrive   Swerve drivetrain to balance
     */
    public TheGreatBalancingAct(SwerveDrivetrain swerveDrive) {
        // Default period is 20 ms
        this(swerveDrive, 0.02, SwerveAutoConstants.kPBalancing);
    }

    @Override
    public void initialize() {

    }

    /**
     * Balances robot by using its pitch and roll as inputs for pid controllers
     * and then sends the values given by the pid controllers to swerve states
     */
    @Override
    public void execute() {
        Rotation3d currentRotation = swerveDrive.getImu().getRotation3d();

        // Calculate turning speed with PID
        double xSpeed = 8 * pitchPidController.calculate(
            currentRotation.getX(), 0
        );
        double ySpeed = -8 * rollPidController.calculate(
            currentRotation.getY(), 0
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
