package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.TankDrivetrain;

public class PreloadRamp extends SequentialCommandGroup {
    public PreloadRamp(TankDrivetrain drive) {
        addCommands(
            RobotContainer.claw.clawOpen()

        );
    }
}
