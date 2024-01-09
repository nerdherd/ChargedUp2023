package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.claw.MotorClaw;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class OnePiecePickup extends SequentialCommandGroup {
    public OnePiecePickup(SwerveAutoBuilder autoBuilder, SwerveDrivetrain swerve, Arm arm, Elevator elevator, MotorClaw claw) {        
        List<PathPlannerTrajectory> pathGroup = PathPlannerAutos.getPathGroup("TwoPiece");

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(180)),
            AutoBuildingBlocks.outtakeHigh(arm, elevator, claw),

            // Turn around
            new TurnToAngle(0, swerve),
            autoBuilder.resetPose(pathGroup.get(0)),

            // Drive over and intake
            Commands.parallel(
                autoBuilder.followPathWithEvents(pathGroup.get(0)),
                AutoBuildingBlocks.moveArm(arm, elevator, ArmConstants.kArmGroundPickup, ElevatorConstants.kElevatorStow),
                claw.setPower(ClawConstants.kIntakePower)
            ),

            // Could be dangerous rotating and stowing at the same time but we'll see
            Commands.parallel(
                new TurnToAngle(180, swerve),
                claw.setPower(ClawConstants.kIntakeNeutralPower),
                AutoBuildingBlocks.stow(arm, elevator)
            ),

            autoBuilder.followPathWithEvents(pathGroup.get(2)),
            AutoBuildingBlocks.outtakeHigh(arm, elevator, claw)
        );
    }
    
}