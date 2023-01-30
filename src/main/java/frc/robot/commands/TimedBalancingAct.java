package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class TimedBalancingAct extends SequentialCommandGroup {
    public TimedBalancingAct(SwerveDrivetrain swerveDrive, double period1, 
                                double kP1, double kP2) {
        super(
            new ParallelRaceGroup(
                new WaitCommand(period1),
                new TheGreatBalancingAct(swerveDrive, kP1)
            ),
            new TheGreatBalancingAct(swerveDrive, kP2)
        );
    }
}
