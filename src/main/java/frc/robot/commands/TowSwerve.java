package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class TowSwerve extends CommandBase {
    private SwerveDrivetrain swerveDrive;

    /**
     * Construct a new TowSwerve command
     * 
     * Tows the Swerve Drive's wheels so that it doesn't move
     * 
     * @param swerveDrive   Swerve drivetrain to rotate
     */
    public TowSwerve(SwerveDrivetrain swerveDrive) {
        this.swerveDrive = swerveDrive;

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
