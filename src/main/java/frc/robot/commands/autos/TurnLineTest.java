package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class TurnLineTest extends SequentialCommandGroup {
    public TurnLineTest(SwerveAutoBuilder autoBuilder, SwerveDrivetrain swerve) {        
        List<PathPlannerTrajectory> pathGroup = PathPlannerAutos.getPathGroup("TurnLineTest");

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(180)),
            autoBuilder.resetPose(pathGroup.get(0)),
            autoBuilder.followPathWithEvents(pathGroup.get(0))
        );
    }
    
}
