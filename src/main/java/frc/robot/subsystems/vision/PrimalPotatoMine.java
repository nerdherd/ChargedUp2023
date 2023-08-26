// package frc.robot.subsystems.vision;

// import edu.wpi.first.wpilibj.Timer;

// public class PrimalPotatoMine {
//     public void initVisionCommands( ) {
//         currentCameraMode = CAMERA_MODE.IDLE;
//         initDoneX = false;
//         initDoneArea = false;
//         xIndex = 0;
//         areaIndex = 0;
//     }

//     Timer timer = new Timer();

//     public void initVisionPickupOnGround(OBJECT_TYPE objType) {
//         initVisionCommands();

//         timer.reset();
//         timer.start();

//         currentGameObject = objType;
//         currentHeightPos = SCORE_POS.LOW;
//         rotationIsNeeded = false; // Reset rotation variable
//         currentLimelight = limelightLow;
//         currentLimelight.setLightState(LightMode.OFF);

//         // This doesn't work for some reason, so we might need to pass the currentGameObject into the drive command directly. (3/11/2023)
//         if (currentGameObject == OBJECT_TYPE.CONE) {
//             goalArea = 2.8; // Alex changed from 3.4 to 2.6 //3.8; // This line is running, so we know the conditional is working (3/11/2023)
//             currentLimelight.setPipeline(1);

//             // Old PID for max 4 m/s
//             PIDArea.setPID(0.4, 0.01, 0.01);
//             PIDTX.setPID(0.05, 0.01, 0.01);
//             PIDYaw.setPID(0, 0, 0);

//             // New PID for max 5 m/s
//             // PIDArea.setPID(0.5, 0, 0.0125);
//             // PIDTX.setPID(0.05, 0, 0.0125);
//             // PIDYaw.setPID(0, 0, 0);
//         } else if (currentGameObject == OBJECT_TYPE.CUBE) {
//             goalArea = 4.1; // Goal area for cube ground pickup // 4.2 OG TODO: DA VINCI RED SIDE WAS CHANGED TO 4.2
//             currentLimelight.setPipeline(2);

//             // PIDArea.setPID(
//             //         SmartDashboard.getNumber("Ta P", 0.75),
//             //         SmartDashboard.getNumber("Ta I", 0.0),
//             //         SmartDashboard.getNumber("Ta D", 0.02)
//             //     );
//             //     PIDTX.setPID(
//             //         SmartDashboard.getNumber("Tx P", 0.05),
//             //         SmartDashboard.getNumber("Tx I", 0.0),
//             //         SmartDashboard.getNumber("Tx D", 0.008)
//             //     );

//             PIDArea.setPID(0.75, 0, 0.02);
//             PIDTX.setPID(0.05, 0, 0.008);
//             PIDYaw.setPID(0, 0, 0);
//         }
//         else if (currentGameObject == OBJECT_TYPE.ATAG) {
//             goalArea = 3.8; 
//             currentLimelight.setPipeline(4);
//         }
//         else {
//             PIDArea.setPID(0, 0, 0);
//             PIDTX.setPID(0, 0, 0);
//             PIDYaw.setPID(0, 0, 0);
//         }
//     }

//     public CommandBase VisionPickupOnGround(OBJECT_TYPE objType) {
//         final PIDController pidAreaFinal = PIDArea;
//         final PIDController pidTXFinal = PIDTX;
//         final PIDController pidYawFinal = PIDYaw;

//         if(limelightLow != null) {
//             return Commands.race(
//                 // Constantly run elevator and arm motion magic
//                 // Commands.run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
//                 // Commands.run(() -> elevator.moveMotionMagic(arm.getArmAngle())),

//                 Commands.sequence(
//                     Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", true)),
//                     Commands.runOnce(() -> initVisionPickupOnGround(objType)),
    
//                     // Move arm and elevator to near ground position in parallel with approaching target
//                     // Commands.deadline(
//                     //     Commands.waitSeconds(2),
//                     //     Commands.parallel( // End command once both arm and elevator have reached their target position
//                     //         Commands.waitUntil(arm.atTargetPosition),
//                     //         Commands.waitUntil(elevator.atTargetPosition),
//                     //         Commands.runOnce(() -> arm.setTargetTicks(-328500)),
//                     //         Commands.runOnce(() -> elevator.setTargetTicks(-36000))
//                     //     ),
//                     //     new RunCommand(() -> driveRotateToTarget(pidAreaFinal, pidTXFinal, pidYawFinal), arm, elevator, claw, drivetrain).until(cameraStatusSupplier)
//                     // ),
                    
    
//                     // Drop arm and elevator so the game piece can be intook
//                     // Commands.race(
//                     //     Commands.waitSeconds(5),
//                     //     Commands.parallel( // End command once both arm and elevator have reached their target position
//                     //         Commands.waitUntil(arm.atTargetPosition),
//                     //         Commands.waitUntil(elevator.atTargetPosition),
//                     //         Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup)),
//                     //         Commands.runOnce(() -> elevator.setTargetTicks(-160000))
//                     //     )
//                     // ),

//                     // Open claw/Start claw intake rollers
//                     claw.setPower(-0.3),
//                     new WaitCommand(.5),
    
//                     // // Close claw/stop claw intake rollers/low background rolling to keep control of game piece
//                     claw.setPower(-0.15),
    
//                     // Stow arm/elev
//                     // Commands.race(
//                     //     Commands.waitSeconds(5),
//                     //     Commands.parallel( // End command once both arm and elevator have reached their target position
//                     //         Commands.waitUntil(arm.atTargetPosition),
//                     //         Commands.waitUntil(elevator.atTargetPosition),
//                     //         Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
//                     //         Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow))
//                     //     )
//                     // ),
                    
//                     Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", false))
//                 )
//             );
            
//         }
//         else {
//             return runOnce(() -> SmartDashboard.putString("Limelight command status:", "Sequence cancelled"));
//         }
//     }
// }
