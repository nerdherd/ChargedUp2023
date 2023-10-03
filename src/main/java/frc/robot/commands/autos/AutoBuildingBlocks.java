package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.claw.MotorClaw;

import static edu.wpi.first.wpilibj2.command.Commands.*;

// NOTE: This is a temporary solution because our arm and elevator code is super out of date
public class AutoBuildingBlocks {
    public static CommandBase moveArm(Arm arm, Elevator elevator, int armPos, int elevatorPos) {
        return 
            race(
                deadline(
                    waitSeconds(2),
                    sequence(
                        runOnce(() -> arm.setTargetTicks(armPos)),
                        waitSeconds(0.1),
                        waitUntil(arm.atTargetPosition)
                    ),
                    sequence(
                        waitSeconds(0.5),
                        runOnce(() -> elevator.setTargetTicks(elevatorPos)),
                        waitSeconds(0.1),
                        waitUntil(elevator.atTargetPosition)
                    )
                ),
                run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
                run(() -> elevator.moveMotionMagic(arm.getArmAngle()))
            );
    }

    public static CommandBase stow(Arm arm, Elevator elevator) {
        return 
            race(
                deadline(
                    waitSeconds(1.1),
                    sequence(
                        waitSeconds(0.5),
                        runOnce(() -> elevator.setTargetTicks(ElevatorConstants.kElevatorStow)),
                        waitSeconds(0.1),
                        waitUntil(elevator.atTargetPosition)
                    ),
                    sequence(
                        waitSeconds(0.9),
                        runOnce(() -> arm.setTargetTicks(ArmConstants.kArmStow)),
                        waitSeconds(0.1),
                        waitUntil(arm.atTargetPosition)
                    )
                ),
                run(() -> arm.moveArmMotionMagic(elevator.percentExtended())),
                run(() -> elevator.moveMotionMagic(arm.getArmAngle()))
            );
    }

    public static CommandBase outtakeHigh(Arm arm, Elevator elevator, MotorClaw claw) {
        return race(
            waitSeconds(5),
            sequence(
                parallel(
                    claw.intake(),
                    moveArm(arm, elevator, ArmConstants.kArmScore, ElevatorConstants.kElevatorScoreHigh)
                ),
                claw.outtake(),
                stow(arm, elevator)
            )
        );
    }

}
