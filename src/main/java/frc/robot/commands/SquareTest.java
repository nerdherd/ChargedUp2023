package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class SquareTest extends SequentialCommandGroup {
    public SquareTest(SwerveAutoBuilder autoBuilder) {
        List<PathPlannerTrajectory> pathGroup = PathPlannerAutos.getPathGroup("TestSquare4");
        
        addCommands(
            Commands.sequence(
                autoBuilder.resetPose(pathGroup.get(0)),
                autoBuilder.followPathWithEvents(pathGroup.get(0)),
                autoBuilder.followPathWithEvents(pathGroup.get(1)),
                autoBuilder.followPathWithEvents(pathGroup.get(2)),
                autoBuilder.followPathWithEvents(pathGroup.get(3))));

    }
    
}
