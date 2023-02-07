// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.Vision.PipelineType;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Claw;
// import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.SwerveDrivetrain;

// public class CommandsThatImplementDriveToTarget {

//     public class getGameObj extends SequentialCommandGroup{
//         public getGameObj(SwerveDrivetrain drivetrain, Limelight limelight, Arm arm, Claw claw, PipelineType pipeline){
//             addCommands(
//                 claw.clawOpen(),
//                 arm.moveArmScore(),
//                 new ApproachCombined(drivetrain, limelight, 2, pipeline),
//                 arm.armExtend(),
//                 claw.clawClose(),
//                 arm.armStow()

//             );
//         }
//     }
//     public class scoreObj extends SequentialCommandGroup{
//         public scoreObj(SwerveDrivetrain drivetrain, Limelight limelight, Arm arm, Claw claw){
//             addCommands(
//                 arm.moveArmScore(),
//                 new ApproachCombined(drivetrain, limelight, 2, PipelineType.TAPE),
//                 arm.armExtend(),
//                 claw.clawOpen(),
//                 arm.armStow()
//             );
//         }
//     }
// }
