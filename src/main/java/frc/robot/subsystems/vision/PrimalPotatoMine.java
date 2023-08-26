package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class PrimalPotatoMine {
    public void initVisionCommands( ) {
        currentCameraMode = CAMERA_MODE.IDLE;
        initDoneX = false;
        initDoneArea = false;
        xIndex = 0;
        areaIndex = 0;
    }

    Timer timer = new Timer();

    public void initVisionPickupOnGround(OBJECT_TYPE objType) { // REMOVE OBJ TYPE PARAMETER HERE AND BELOW TOO
        initVisionCommands();

        timer.reset();
        timer.start();

        currentLimelight.setLightState(LightMode.OFF);

        if (currentGameObject == OBJECT_TYPE.CUBE) {
            goalArea = 4.1; // Goal area for cube ground pickup // 4.2 OG TODO: DA VINCI RED SIDE WAS CHANGED TO 4.2
            currentLimelight.setPipeline(2);

            // PIDArea.setPID(
            //         SmartDashboard.getNumber("Ta P", 0.75),
            //         SmartDashboard.getNumber("Ta I", 0.0),
            //         SmartDashboard.getNumber("Ta D", 0.02)
            //     );
            //     PIDTX.setPID(
            //         SmartDashboard.getNumber("Tx P", 0.05),
            //         SmartDashboard.getNumber("Tx I", 0.0),
            //         SmartDashboard.getNumber("Tx D", 0.008)
            //     );

            PIDArea.setPID(0.75, 0, 0.02);
            PIDTX.setPID(0.05, 0, 0.008);
            PIDYaw.setPID(0, 0, 0);
        }
        }
        else {
            PIDArea.setPID(0, 0, 0);
            PIDTX.setPID(0, 0, 0);
            PIDYaw.setPID(0, 0, 0);
        }
    }

    public CommandBase VisionPickupOnGround(OBJECT_TYPE objType) { // REMOVE OBJ TYPE PARAMETER
        final PIDController pidAreaFinal = PIDArea;
        final PIDController pidTXFinal = PIDTX;
        final PIDController pidYawFinal = PIDYaw;

        if(limelightLow != null) {
            return Commands.race(
                // Constantly run elevator and arm motion magic
                // Commands.run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
                // Commands.run(() -> elevator.moveMotionMagic(arm.getArmAngle())),

                Commands.sequence(
                    Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", true)),
                    Commands.runOnce(() -> initVisionPickupOnGround(objType)),
    
                    // Move arm and elevator to near ground position in parallel with approaching target
                    // Commands.deadline(
                    //     Commands.waitSeconds(2),
                    //     Commands.parallel( // End command once both arm and elevator have reached their target position
                    //         Commands.waitUntil(arm.atTargetPosition),
                    //         Commands.waitUntil(elevator.atTargetPosition),
                    //         Commands.runOnce(() -> arm.setTargetTicks(-328500)),
                    //         Commands.runOnce(() -> elevator.setTargetTicks(-36000))
                    //     ),
                    //     new RunCommand(() -> driveRotateToTarget(pidAreaFinal, pidTXFinal, pidYawFinal), arm, elevator, claw, drivetrain).until(cameraStatusSupplier)
                    // ),
                    
    
                    // Drop arm and elevator so the game piece can be intook
                    // Commands.race(
                    //     Commands.waitSeconds(5),
                    //     Commands.parallel( // End command once both arm and elevator have reached their target position
                    //         Commands.waitUntil(arm.atTargetPosition),
                    //         Commands.waitUntil(elevator.atTargetPosition),
                    //         Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmGroundPickup)),
                    //         Commands.runOnce(() -> elevator.setTargetTicks(-160000))
                    //     )
                    // ),

                    // Open claw/Start claw intake rollers
                    claw.setPower(-0.3),
                    new WaitCommand(.5),
    
                    // // Close claw/stop claw intake rollers/low background rolling to keep control of game piece
                    claw.setPower(-0.15),
    
                    // Stow arm/elev
                    // Commands.race(
                    //     Commands.waitSeconds(5),
                    //     Commands.parallel( // End command once both arm and elevator have reached their target position
                    //         Commands.waitUntil(arm.atTargetPosition),
                    //         Commands.waitUntil(elevator.atTargetPosition),
                    //         Commands.runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                    //         Commands.runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow))
                    //     )
                    // ),
                    
                    Commands.runOnce(() -> SmartDashboard.putBoolean("Vision Pickup Running", false))
                )
            );
            
        }
        else {
            return runOnce(() -> SmartDashboard.putString("Limelight command status:", "Sequence cancelled"));
        }
    }
}
