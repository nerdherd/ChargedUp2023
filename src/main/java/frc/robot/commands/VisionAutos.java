package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.PipelineType;

public class VisionAutos {

    public static CommandBase penPineappleApplePen(SwerveDrivetrain drivetrain, Vision vision) {
        return Commands.parallel(
            new InstantCommand(() -> SmartDashboard.putBoolean("VisonAuto called", true)),
            new RunCommand(() -> vision.getPPAP(drivetrain))
        );
        //return new RunCommand(() -> getPPAP(drivetrain, limelight));
        // return new DriveToTarget(drivetrain, limelight, 2, PipelineType.CONE);
    }


}
