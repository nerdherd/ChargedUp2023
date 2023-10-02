package frc.robot.commands.oldautos;
// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.subsystems.swerve.SwerveDrivetrain;
// import frc.robot.subsystems.vision.VROOOOM;

// public class VisionDriveCommands {
//     //Why do programmers have good vision?
//     // they can C++ :)))

//     public static CommandBase highCOneCOneNUT(VROOOOM vision, SwerveDrivetrain swerve){
//         PIDController pidArea = new PIDController(0, 0, 0);
//         PIDController pidTx = new PIDController(0, 0, 0);
//         PIDController pidYaw = new PIDController(0, 0, 0);

//         vision.setLimelightHigh();
//         vision.setLimelightPipeline(1);
//         vision.setGoalArea(21);
//         vision.setGoalTX(0);
//         vision.setGoalYaw(0);

//         return Commands.run(() -> vision.driveRotateToTarget(pidArea, pidTx, pidYaw), swerve);
//     }
//     public static CommandBase lowCOneCOneNUT(VROOOOM vision, SwerveDrivetrain swerve){
//         PIDController pidArea = new PIDController(0, 0, 0);
//         PIDController pidTx = new PIDController(0, 0, 0);

//         vision.setLimelightLow();
//         vision.setLimelightPipeline(1);
//         vision.setGoalArea(21);
//         vision.setGoalTX(0);

//         return Commands.run(() -> vision.skrttttToTarget(pidArea, pidTx), swerve);
//     }

//     public static CommandBase highCubeRoot(VROOOOM vision, SwerveDrivetrain swerve){
//         PIDController pidArea = new PIDController(0, 0, 0);
//         PIDController pidTx = new PIDController(0, 0, 0);
//         PIDController pidYaw = new PIDController(0, 0, 0);

//         vision.setLimelightHigh();
//         vision.setLimelightPipeline(2);
//         vision.setGoalArea(0);
//         vision.setGoalTX(0);
//         vision.setGoalYaw(0);

//         return Commands.run(() -> vision.driveRotateToTarget(pidArea, pidTx, pidYaw), swerve);
//     }
//     public static CommandBase lowCubeRoot(VROOOOM vision, SwerveDrivetrain swerve){
//         PIDController pidArea = new PIDController(0, 0, 0);
//         PIDController pidTx = new PIDController(0, 0, 0);

//         vision.setLimelightLow();
//         vision.setLimelightPipeline(2);
//         vision.setGoalArea(0);
//         vision.setGoalTX(0);

//         return Commands.run(() -> vision.skrttttToTarget(pidArea, pidTx), swerve);
//     }

//     public static CommandBase highMixTape(VROOOOM vision, SwerveDrivetrain swerve){
//         PIDController pidArea = new PIDController(7, 0, 0);
//         PIDController pidTx = new PIDController(0.1, 0, 0.1);
//         PIDController pidYaw = new PIDController(0.1, 0, 0);

//         vision.setLimelightHigh();
//         vision.setLimelightPipeline(3);
//         vision.setGoalArea(0.15);
//         vision.setGoalTX(0);
//         vision.setGoalYaw(180);

//         return Commands.run(() -> vision.driveRotateToTarget(pidArea, pidTx, pidYaw), swerve);
//     }
//     public static CommandBase lowMixTape(VROOOOM vision, SwerveDrivetrain swerve){
//         PIDController pidArea = new PIDController(7, 0, 0);
//         PIDController pidTx = new PIDController(0.1, 0, 0.1);
//         PIDController pidYaw = new PIDController(0.1, 0, 0);

//         vision.setLimelightLow();
//         vision.setLimelightPipeline(3);
//         vision.setGoalArea(0.15);
//         vision.setGoalTX(0);
//         vision.setGoalYaw(180);

//         return Commands.run(() -> vision.driveRotateToTarget(pidArea, pidTx, pidYaw), swerve);
//     }

//     public static CommandBase gamerTag(VROOOOM vision, SwerveDrivetrain swerve){
//         PIDController pidArea = new PIDController(7, 0, 0);
//         PIDController pidTx = new PIDController(0.1, 0, 0.1);
//         PIDController pidYaw = new PIDController(0.1, 0, 0);

//         vision.setLimelightLow();
//         vision.setLimelightPipeline(4);
//         vision.setGoalArea(2);
//         vision.setGoalTX(0);
//         vision.setGoalYaw(180);

//         return Commands.run(() -> vision.driveRotateToTarget(pidArea, pidTx, pidYaw), swerve);
//     }

    
// }
