package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

public class PreloadTaxi extends SequentialCommandGroup {
    
    public PreloadTaxi(Drivetrain drive, Claw claw, Arm arm) {
        addCommands(
            claw.clawClose(),
            // arm.armToMiddleNodePosition(),
            claw.clawOpen(),
            arm.armStow(),
            new WaitCommand(2),
            new InstantCommand(() -> drive.setPower(-0.5, -0.5)),
            new WaitCommand(3),
            new InstantCommand(() -> drive.setPower(0, 0))
        );
    }
}