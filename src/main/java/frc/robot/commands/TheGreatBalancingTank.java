package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TankAutoConstants;
import frc.robot.subsystems.TankDrivetrain;

public class TheGreatBalancingTank extends CommandBase {
    private TankDrivetrain tankDrive;
    private PIDController pitchPidController;

    /**
     * Construct a new TurnToAngle Command
     * @param swerveDrive   Swerve drivetrain to rotate
     * @param period      
     *   Time between each calculation (default 20ms)
     */
    public TheGreatBalancingTank(TankDrivetrain tankDrive, double period) {
        this.tankDrive = tankDrive;
        
        this.pitchPidController = new PIDController(
            TankAutoConstants.kPBalancing, 0, 
            TankAutoConstants.kDBalancing, period);
        
        this.pitchPidController.enableContinuousInput(0, 360);
        addRequirements(tankDrive);
    }

    public TheGreatBalancingTank(TankDrivetrain tankDrive) {
        // Default period is 20 ms
        this(tankDrive, 0.02);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Rotation3d currentRotation = tankDrive.getRotation3d();

        // Calculate turning speed with PID
        double xSpeed = pitchPidController.calculate(
            currentRotation.getY(), 0
        );

        // Convert to percent
        xSpeed /= 90;

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("pitch", currentRotation.getY());
        SmartDashboard.putNumber("yaw", currentRotation.getZ());

        // Convert speed into swerve states
        tankDrive.drive(xSpeed, xSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
