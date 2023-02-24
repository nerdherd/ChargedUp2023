package frc.robot.legacy;
// package frc.robot.commands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.swerve.SwerveDrivetrain;
// import frc.robot.subsystems.vision.Vision;

// public class VisionCommands {

//     // seekConeOnGroundToPickup
//     public static CommandBase penPineappleApplePen(SwerveDrivetrain drivetrain, Vision vision) {
//         return Commands.parallel(
//             new InstantCommand(() -> SmartDashboard.putBoolean("VisonAuto1 ended", false)),
//             new SequentialCommandGroup(
//                 Commands.runOnce(() -> vision.initVisionCommands()),
//                 new ParallelRaceGroup(
//                     new RunCommand(() -> vision.getPPAP(drivetrain)),
//                     Commands.waitUntil(vision.cameraStatusSupplier),
//                     new WaitCommand(3000) //TODO debug
//                 ),
//                 new InstantCommand(() -> SmartDashboard.putBoolean("VisonAuto1 ended", true))
//             )
//         );
//     }

//     public static CommandBase seekTapeDropCone(SwerveDrivetrain drivetrain, Vision vision) {
//         return Commands.parallel(
//             new InstantCommand(() -> SmartDashboard.putBoolean("VisonAuto2 ended", false)),
//             new SequentialCommandGroup(
//                 Commands.runOnce(() -> vision.initVisionCommands()),
//                 new ParallelRaceGroup(
//                     new RunCommand(() -> vision.seekTape(drivetrain)),
//                     Commands.waitUntil(vision.cameraStatusSupplier),
//                     new WaitCommand(3000) //DODO debug
//                 ),
//                 new InstantCommand(() -> SmartDashboard.putBoolean("VisonAuto2 ended", true))
//             )
//         );
//     }

//     public static CommandBase FindYourselfATag(SwerveDrivetrain drivetrain, Vision vision) {
//         return Commands.parallel(
//             new InstantCommand(() -> SmartDashboard.putBoolean("VisonAuto3 ended", false)),
//             new SequentialCommandGroup(
//                 Commands.runOnce(() -> vision.initVisionCommands()),
//                 new ParallelRaceGroup(
//                     new RunCommand(() -> vision.seekATag(drivetrain)),
//                     Commands.waitUntil(vision.cameraStatusSupplier),
//                     new WaitCommand(3000) //DODO debug
//                 ),
//                 new InstantCommand(() -> SmartDashboard.putBoolean("VisonAuto3 ended", true))
//             )
//         );
//     }

// }
