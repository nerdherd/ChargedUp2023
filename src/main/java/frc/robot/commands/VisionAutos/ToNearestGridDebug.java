package frc.robot.commands.VisionAutos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.primalWallnut.PrimalSunflower;

public class ToNearestGridDebug extends CommandBase {
    private SwerveDrivetrain swerve;
    private PrimalSunflower vision;

    /**
     * Construct a new ToNearestGrid command
     * 
     * Drive the robot to the nearest grid using odometry.
     * 
     * @param swerveDrive   Swerve drivetrain to move
     */
    public ToNearestGridDebug(SwerveDrivetrain swerve, PrimalSunflower vision) {
        this.swerve = swerve;
        this.vision = vision;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        vision.toNearestGridDebug(swerve);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}