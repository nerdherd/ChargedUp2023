// package frc.robot.commands;

// import java.util.List;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.ElevatorConstants;
// import frc.robot.Constants.SwerveAutoConstants;
// import frc.robot.Constants.SwerveDriveConstants;
// import frc.robot.commands.SwerveAutos.StartPosition;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.claw.MotorClaw;
// import frc.robot.subsystems.swerve.SwerveDrivetrain;
// import frc.robot.subsystems.vision.VROOOOM;
// import frc.robot.subsystems.vision.VROOOOM.OBJECT_TYPE;
// import frc.robot.subsystems.vision.VROOOOM.SCORE_POS;

// import static frc.robot.Constants.SwerveAutoConstants.*;
// import static edu.wpi.first.wpilibj2.command.Commands.*;

// // Trajectory must stop ~32 inches in front of the cone
// // Trajectory can stop anywhere in front of the tape/tag as long as the robot is in front of the charging station
// // All score positions are assumed to be high for this file right now
// // Positive Y = left, positive X = towards drivers, rotation positive is clockwise

// public class VisionAllLowAuto {
    
//     public static CommandBase ThreeCubesAuto(SwerveDrivetrain swerveDrive, VROOOOM vision, Arm arm, Elevator elevator, MotorClaw claw, 
//     Alliance alliance){
//         PIDController trajectoryXController = new PIDController(kPXController, kIXController, kDXController);
//         PIDController trajectoryYController = new PIDController(kPYController, kIYController, kDYController);
//         ProfiledPIDController trajectoryThetaController = 
//             new ProfiledPIDController(kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);

//         // Create trajectory settings
//         TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
//             SwerveAutoConstants.kMaxSpeedMetersPerSecond, SwerveAutoConstants.kMaxAccelerationMetersPerSecondSquared);

//         double zoooomAllianceThingy = 1.0;
//         if (alliance == Alliance.Red) {
//             zoooomAllianceThingy = -1.0;
//         }
        
//         //trajectory stuff

//         Trajectory zoooomToCube = TrajectoryGenerator.generateTrajectory(
//             new Pose2d(0, 0, new Rotation2d(0)), 
//             List.of(
//                 new Translation2d(0.1, 0 * zoooomAllianceThingy),
//                 new Translation2d(-0.1, 0 * zoooomAllianceThingy), // push the cube to hybrid zone
//                 new Translation2d(0.18, 0.18 * zoooomAllianceThingy) 
//                 //new Translation2d(-1.8, -0.4)
//             ),
//             new Pose2d(4.4, 0.18 * zoooomAllianceThingy, Rotation2d.fromDegrees(0)),
//             trajectoryConfig);

//         Trajectory cubeToZoooom = TrajectoryGenerator.generateTrajectory(
//             List.of(
//                 new Pose2d(3.6, 0.1 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)), // TODO: Run with and without this line
//                 new Pose2d(1.5, 0.1 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)),
//                 new Pose2d(0, 0.1 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)),
//                 new Pose2d(0, 0.61 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)),
//                 new Pose2d(-0.5, 0.61 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9))
//             ),
//             trajectoryConfig);

//         Trajectory zoooomPartTwo = TrajectoryGenerator.generateTrajectory(
//             List.of(
//                 //new Pose2d(0.2, 1.0, Rotation2d.fromDegrees(179.9)),
//                 new Pose2d(0, 0.1 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)), // tested, somehow the trajectory 0,0 was shifted
//                 new Pose2d(0, -0.3 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)),
//                 new Pose2d(4.2, -0.3 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9))
//                 // new Pose2d(3.6, 2.2 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9))
//             ),
//             trajectoryConfig);

        

//         SwerveControllerCommand zoooomToCubeCommand = new SwerveControllerCommand(
//             zoooomToCube, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
//             trajectoryXController, trajectoryYController, trajectoryThetaController, swerveDrive::setModuleStates, swerveDrive);

//         SwerveControllerCommand cubeToZoooomCommand = new SwerveControllerCommand(
//             cubeToZoooom, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
//             trajectoryXController, trajectoryYController, trajectoryThetaController, swerveDrive::setModuleStates, swerveDrive);

//         SwerveControllerCommand zoooomPartTwoCommand = new SwerveControllerCommand(
//             zoooomPartTwo, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
//             trajectoryXController, trajectoryYController, trajectoryThetaController, swerveDrive::setModuleStates, swerveDrive);


//         return Commands.race(
//             Commands.waitSeconds(15), // TODO DEL
//             //init
//             Commands.sequence(
//                 Commands.parallel(
//                     Commands.runOnce(() -> swerveDrive.resetOdometry(zoooomToCube.getInitialPose()))
//                     //claw.setPower(-0.5) for shoot out cone
//                 ),
//                 //Commands.waitSeconds(0.1),for shoot out cone

//                 //trajectory to cube
//                 Commands.parallel(
//                     zoooomToCubeCommand,
//                     //claw.setPower(0),for shoot out cone
    
//                     //Drop arm to half way
//                     Commands.deadline( // TODO: Fix this and the other two Deadline arm Commands
//                         Commands.waitSeconds(0.5),
//                         sequence(
//                             runOnce(() -> arm.setTargetTicks((ArmConstants.kArmScore + ArmConstants.kArmGroundPickup) / 2)),
//                             waitSeconds(0.5),
//                             waitUntil(arm.atTargetPosition)
//                         )
//                     ),
                    
//                     Commands.runOnce(() -> vision.initVisionPickupOnGround(OBJECT_TYPE.CUBE))
//                 ),
//                 Commands.runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
//                 Commands.runOnce(() -> swerveDrive.stopModules()),

//                 Commands.race(
//                     new RunCommand(() -> vision.driveToCubeOnGround(claw, 5), arm, elevator, claw, swerveDrive).until(vision.cameraStatusSupplier),
//                     Commands.waitSeconds(20) // kill this auto
//                     // TODO need add protection here!!!!!!
//                 ),
//                 Commands.runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
//                 Commands.runOnce(() -> swerveDrive.stopModules()),

//                 claw.setPower(-0.35),
//                 Commands.deadline(
//                     Commands.waitSeconds(0.5),
//                     sequence(
//                         runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup)),
//                         waitSeconds(0.5),
//                         waitUntil(arm.atTargetPosition)
//                     )
//                 ),
//                 // Close claw/stop claw intake rollers/low background rolling to keep control of game piece
//                 claw.setPower(-0.20),

//                 Commands.deadline(
//                     Commands.waitSeconds(0.5),
//                     sequence(
//                         runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
//                         waitSeconds(0.5),
//                         waitUntil(arm.atTargetPosition)
//                     )
//                 ),
                
//                 new TurnToAngle(-179.9, swerveDrive),

//                 cubeToZoooomCommand,
//                 Commands.runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
//                 Commands.runOnce(() -> swerveDrive.stopModules()),

//                 // TODO: apriltag?

//                 claw.setPower(0.3),

//                 Commands.parallel(
//                     Commands.sequence(
//                         waitSeconds(0.2),
//                         claw.setPower(0)
//                     ),
//                     zoooomPartTwoCommand,
//                     Commands.runOnce(() -> vision.initVisionPickupOnGround(OBJECT_TYPE.CUBE))
//                 ),
//                 Commands.runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
//                 Commands.runOnce(() -> swerveDrive.stopModules()),

//                 new TurnToAngle(-45, swerveDrive),

//                 /* uncommment it if we have time to run, otherwise be safe to stop here
//                 Commands.race(
//                     new RunCommand(() -> vision.driveToCubeOnGround(claw, 5), arm, elevator, claw, swerveDrive).until(vision.cameraStatusSupplier),
//                     Commands.waitSeconds(20)
//                     // TODO need add protection here!!!!!!
//                 ),*/
//                 Commands.runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates)),
//                 Commands.runOnce(() -> swerveDrive.stopModules())
//             ),

//             run(() -> arm.moveArmMotionMagic(elevator.percentExtended()))
//         );
//     }





























//     public static CommandBase ThreeCubesAutoFast(SwerveDrivetrain swerveDrive, VROOOOM vision, Arm arm, Elevator elevator, MotorClaw claw, 
//     Alliance alliance){
//         PIDController trajectoryXController = new PIDController(kPXController, kIXController, kDXController);
//         PIDController trajectoryYController = new PIDController(kPYController, kIYController, kDYController);
//         ProfiledPIDController trajectoryThetaController = 
//             new ProfiledPIDController(kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);

//         // Create trajectory settings
//         TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
//             SwerveAutoConstants.kMaxSpeedMetersPerSecond, SwerveAutoConstants.kMaxAccelerationMetersPerSecondSquared);

//         double zoooomAllianceThingy = 1.0;
//         if (alliance == Alliance.Red) {
//             zoooomAllianceThingy = -1.0;
//         }
        
//         //trajectory stuff

//         Trajectory zoooomToCube = TrajectoryGenerator.generateTrajectory(
//             new Pose2d(0, 0, new Rotation2d(0)), 
//             List.of(
//                 new Translation2d(0.18, 0 * zoooomAllianceThingy),
//                 new Translation2d(-0.18, 0 * zoooomAllianceThingy), // push the cube to hybrid zone
//                 new Translation2d(0, 0 * zoooomAllianceThingy),
//                 new Translation2d(0.18, 0.18 * zoooomAllianceThingy) 
//                 //new Translation2d(-1.8, -0.4)
//             ),
//             new Pose2d(4.3, 0.18 * zoooomAllianceThingy, Rotation2d.fromDegrees(0)),
//             trajectoryConfig);

//         Trajectory cubeToZoooom = TrajectoryGenerator.generateTrajectory(
//             List.of(
//                 new Pose2d(4.3, 0.18 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)),
//                 new Pose2d(3.8, -0.4 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)), 
//                 new Pose2d(1.5, -0.4 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)),
//                 new Pose2d(-0.3, -0.4 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)),
//                 new Pose2d(-0.3, 0.4 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)),
//                 new Pose2d(-0.45, 0.4 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9))
//             ),
//             trajectoryConfig);

//         Trajectory zoooomPartTwo = TrajectoryGenerator.generateTrajectory(
//             List.of(
//                 //new Pose2d(0.2, 1.0, Rotation2d.fromDegrees(179.9)),
//                 new Pose2d(0, 0 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)), // tested, somehow the trajectory 0,0 was shifted
//                 new Pose2d(0.5, -0.5 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9)),
//                 new Pose2d(4.2, -0.5 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9))
//                 // new Pose2d(3.6, 2.2 * zoooomAllianceThingy, Rotation2d.fromDegrees(179.9))
//             ),
//             trajectoryConfig);

        

//         SwerveControllerCommand zoooomToCubeCommand = new SwerveControllerCommand(
//             zoooomToCube, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
//             trajectoryXController, trajectoryYController, trajectoryThetaController, swerveDrive::setModuleStates, swerveDrive);

//         SwerveControllerCommand cubeToZoooomCommand = new SwerveControllerCommand(
//             cubeToZoooom, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
//             trajectoryXController, trajectoryYController, trajectoryThetaController, swerveDrive::setModuleStates, swerveDrive);

//         SwerveControllerCommand zoooomPartTwoCommand = new SwerveControllerCommand(
//             zoooomPartTwo, swerveDrive::getPose, SwerveDriveConstants.kDriveKinematics, 
//             trajectoryXController, trajectoryYController, trajectoryThetaController, swerveDrive::setModuleStates, swerveDrive);


//         return Commands.race(
//             Commands.waitSeconds(15), // TODO DEL
//             //init
//             Commands.sequence(

//                 Commands.runOnce(() -> swerveDrive.resetOdometry(zoooomToCube.getInitialPose())),

//                 //trajectory to cube
//                 Commands.parallel(
//                     zoooomToCubeCommand,
//                     runOnce(() -> arm.setTargetTicks((ArmConstants.kArmScore + ArmConstants.kArmGroundPickup) / 2)),
//                     Commands.runOnce(() -> vision.initVisionPickupOnGround(OBJECT_TYPE.CUBE))
//                 ),

//                 Commands.race(
//                     new RunCommand(() -> vision.driveToCubeOnGround(claw, 5), arm, elevator, claw, swerveDrive).until(vision.cameraStatusSupplier),
//                     Commands.waitSeconds(20) // kill this auto
//                     // TODO need add protection here!!!!!!
//                 ),

//                 Commands.parallel(
//                     claw.setPower(-0.36),
//                     Commands.deadline(
//                         Commands.waitSeconds(0.6),
//                         sequence(
//                             runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup)),
//                             waitUntil(arm.atTargetPosition)
//                         )
//                     )
//                 ),

//                 Commands.waitSeconds(0.1),

//                 Commands.parallel(
//                     // Close claw/stop claw intake rollers/low background rolling to keep control of game piece
//                     claw.setPower(-0.20),
//                     runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
//                     Commands.sequence(
//                         waitSeconds(0.2),
//                         new TurnToAngle(-179.9, swerveDrive) // it needs zoooomAllianceThingy? TODO
//                     )
//                 ),

//                 cubeToZoooomCommand,

//                 claw.setPower(0.3),

//                 Commands.parallel(
//                     Commands.sequence(
//                         waitSeconds(0.2),
//                         claw.setPower(0)
//                     ),
//                     zoooomPartTwoCommand,
//                     Commands.runOnce(() -> vision.initVisionPickupOnGround(OBJECT_TYPE.CUBE))
//                 ),

//                 new TurnToAngle(45 * zoooomAllianceThingy, swerveDrive),
                
//                 // Commands.race(
//                 //     new RunCommand(() -> vision.driveToCubeOnGround(claw, 2), arm, elevator, claw, swerveDrive).until(vision.cameraStatusSupplier),
//                 //     Commands.waitSeconds(5) // kill this auto
//                 // ),

//                 Commands.parallel(
//                     Commands.runOnce(() -> swerveDrive.stopModules())
//                     // claw.setPower(-0.36),
//                     // runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup))
//                 ),
//                 Commands.runOnce(() -> swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates))

//                 // Commands.parallel(
//                 //     // Close claw/stop claw intake rollers/low background rolling to keep control of game piece
//                 //     claw.setPower(-0.20),
//                 //     runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow))
//                 // )
//             ),

//             run(() -> arm.moveArmMotionMagic(elevator.percentExtended()))
//         );
//     }
// }
