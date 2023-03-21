package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class TimedBalancingAct extends SequentialCommandGroup {
    /**
     * Uses two kp values to make balancing act faster then slower as it approaches target
     * has to be manual because charging station's tipping isn't gradual
     * @param swerveDrive   swerve drivetrain to balance
     * @param period1       length of time kp1 is used before switching to kp2
     * @param kP1           higher p value for pid controller
     * @param kP2           lower p value for pid controller
     */
    @Deprecated
    public TimedBalancingAct(SwerveDrivetrain swerveDrive, double period1, 
                                double kP1, double kP2) {
        super(
            new ParallelRaceGroup(
                new WaitCommand(period1),
                new TheGreatBalancingAct(swerveDrive, kP1, 0, 0)
            ),
            new TheGreatBalancingAct(swerveDrive, kP2, 0, 0)
        );
    }
}
