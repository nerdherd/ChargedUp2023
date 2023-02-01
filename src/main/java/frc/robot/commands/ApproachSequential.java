package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;

public class ApproachSequential extends SequentialCommandGroup {
    public ApproachSequential(Limelight limelight, SwerveDrivetrain drivetrain){
        addCommands(
            new TurnToAngle(180, drivetrain),
            new DriveToTarget(limelight, drivetrain, 0.2)
        );
    }
}
