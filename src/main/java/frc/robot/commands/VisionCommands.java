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
import frc.robot.subsystems.vision.Vision.VisionTask;

public class VisionCommands {

    // seekConeOnGroundToPickup
    public static CommandBase penPineappleApplePen(SwerveDrivetrain drivetrain, Vision vision) {
        return Commands.parallel(
            new InstantCommand(() -> SmartDashboard.putBoolean("VisonAuto1 ended", false)),
            new SequentialCommandGroup(
                Commands.runOnce(() -> vision.initObjDetection(VisionTask.CONE_ON_GROUND, 21, 179)),
                new ParallelRaceGroup(
                    new RunCommand(() -> vision.getPPAP(drivetrain)),
                    Commands.waitUntil(vision.cameraLowStatusSupplier),
                    new WaitCommand(3000) //TODO debug
                ),
                new InstantCommand(() -> SmartDashboard.putBoolean("VisonAuto1 ended", true))
            )
        );
    }

    public static CommandBase seekTapeDropCone(SwerveDrivetrain drivetrain, Vision vision) {
        return Commands.parallel(
            new InstantCommand(() -> SmartDashboard.putBoolean("VisonAuto2 ended", false)),
            new SequentialCommandGroup(
                Commands.runOnce(() -> vision.initObjDetection(VisionTask.TAPE_ON_POLE, 0.15, 0)),
                new ParallelRaceGroup(
                    new RunCommand(() -> vision.seekTape(drivetrain)),
                    Commands.waitUntil(vision.cameraHighStatusSupplier),
                    new WaitCommand(3000) //DODO debug
                ),
                new InstantCommand(() -> SmartDashboard.putBoolean("VisonAuto2 ended", true))
            )
        );
    }

    public static CommandBase FindYourselfATag(SwerveDrivetrain drivetrain, Vision vision) {
        return Commands.parallel(
            new InstantCommand(() -> SmartDashboard.putBoolean("VisonAuto3 ended", false)),
            new SequentialCommandGroup(
                Commands.runOnce(() -> vision.initObjDetection(VisionTask.ATAG_GRID, 2, 0)),
                new ParallelRaceGroup(
                    new RunCommand(() -> vision.seekATag(drivetrain)),
                    Commands.waitUntil(vision.cameraHighStatusSupplier),
                    new WaitCommand(3000) //DODO debug
                ),
                new InstantCommand(() -> SmartDashboard.putBoolean("VisonAuto3 ended", true))
            )
        );
    }

}
