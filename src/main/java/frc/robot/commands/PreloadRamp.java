package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class PreloadRamp extends SequentialCommandGroup {
    public PreloadRamp(Drivetrain drive) {
        addCommands(
            new InstantCommand(() -> drive.backwardDistance(10)),
            // arm.armToTopNodePosition(),
            // arm.armStow(),
            new InstantCommand(() -> drive.forwardDistance(10))
            // new Balance()
        );
    }
}
