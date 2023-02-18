package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;

public class VisionCommands {

    // Approach cone without rotation
    public static CommandBase penPineappleApplePen(SwerveDrivetrain drivetrain, Vision vision) {
        return Commands.parallel(
            new InstantCommand(() -> SmartDashboard.putBoolean("VisonAuto1 ended", false)),
            new SequentialCommandGroup(
                Commands.runOnce(() -> vision.initObjDetection(false, 6, 180)),
                new ParallelRaceGroup(
                    new RunCommand(() -> vision.getPPAP(drivetrain)),
                    Commands.waitUntil(vision.cameraLowStatusSupplier),
                    new WaitCommand(30)
                ),
                new InstantCommand(() -> SmartDashboard.putBoolean("VisonAuto1 ended", true))
            )
        );
    }

    public static CommandBase seekTapeDropCone(SwerveDrivetrain drivetrain, Vision vision) {
        return Commands.parallel(
            new InstantCommand(() -> SmartDashboard.putBoolean("VisonAuto2 ended", false)),
            new SequentialCommandGroup(
                Commands.runOnce(() -> vision.initObjDetection(true, 0.3, 0)),
                new ParallelRaceGroup(
                    new RunCommand(() -> vision.seekTape(drivetrain)),
                    Commands.waitUntil(vision.cameraHighStatusSupplier),
                    new WaitCommand(30)
                ),
                new InstantCommand(() -> SmartDashboard.putBoolean("VisonAuto2 ended", true))
            )
        );
    }

}
