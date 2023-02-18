package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.PipelineType;

public class VisionAutos {

    // seekConeOnGroundToPickup
    public static CommandBase penPineappleApplePen(SwerveDrivetrain drivetrain, Vision vision) {
        return Commands.parallel(
            new InstantCommand(() -> SmartDashboard.putBoolean("VisonAuto1 ended", false)),
            new SequentialCommandGroup(
                Commands.runOnce(() -> vision.initObjDetection(false, 21, 180)),
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
                Commands.runOnce(() -> vision.initObjDetection(true, 0.15, 0)),
                new ParallelRaceGroup(
                    new RunCommand(() -> vision.seekTape(drivetrain)),
                    Commands.waitUntil(vision.cameraHighStatusSupplier),
                    new WaitCommand(3000) //DODO debug
                ),
                new InstantCommand(() -> SmartDashboard.putBoolean("VisonAuto2 ended", true))
            )
        );
    }

}
