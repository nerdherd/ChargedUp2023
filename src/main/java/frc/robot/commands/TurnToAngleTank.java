package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TankDrivetrain;

public class TurnToAngleTank extends CommandBase{
    private double targetAngle;
    private TankDrivetrain drivetrain;
    private PIDController pidController;

    /**
     * Construct a new TurnToAngle Command
     * @param targetAngle   Target angle (degrees)
     * @param swerveDrive   Swerve drivetrain to rotate
     * @param period        Time between each calculation (default 20ms)
     */
    public TurnToAngleTank(double targetAngle, TankDrivetrain drivetrain, double period) {
        this.targetAngle = targetAngle;
        this.drivetrain = drivetrain;

        this.pidController = new PIDController(0.1, 0, 0, period);
        
        this.pidController.setTolerance(4, 2 * period);
        
        this.pidController.enableContinuousInput(0, 360);
        
        addRequirements(drivetrain);
    }

    public TurnToAngleTank(TankDrivetrain drivetrain, double targetAngle) {
        // Default period is 20 ms
        this(targetAngle, drivetrain, 0.02);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Calculate turning speed with PID
        double turningSpeed = pidController.calculate(drivetrain.getHeading(), targetAngle);
        
        //swerveDrive.setModuleStates(moduleStates);
        drivetrain.tankDrive(turningSpeed, turningSpeed * -1);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}