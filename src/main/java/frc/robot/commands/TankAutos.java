package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.TankDrivetrain;

public class TankAutos {
    
    public static CommandBase HardCarryAuto(TankDrivetrain drivetrain, Claw claw, Arm arm) {
        return Commands.sequence(
            arm.armExtend(),
            claw.clawOpen(),
            arm.armStow(),
            new DriveStraight(drivetrain, 0.5, -0.762, 0, false),
            new TurnToAngleTank(drivetrain, -90),
            new DriveStraight(drivetrain, 0.5, 0.4064, 0, false),
            new TurnToAngleTank(drivetrain, -90),
            new DriveStraight(drivetrain, 0.5, 5.585, 0, false),
            claw.clawClose(),
            new DriveStraight(drivetrain, 0.5, -5.585, 0, false),
            new TurnToAngleTank(drivetrain, 90),
            new DriveStraight(drivetrain, 0.5, 0.4064, 0, false),
            new TurnToAngleTank(drivetrain, 90),
            new DriveStraight(drivetrain, 0.5, 0.762, 0, false),
            arm.armExtend(),
            claw.clawOpen(),
            arm.armStow(),
            new DriveStraight(drivetrain, -0.5, -0.762, 0, false),
            new TurnToAngleTank(drivetrain, -90),
            new DriveStraight(drivetrain, 0.5, 0.6, 0, false),
            new TurnToAngleTank(drivetrain, -90),
            new DriveStraight(drivetrain, 0.5, 1.238, 0, false)
            );
    }

    public static CommandBase DietCokeAuto(TankDrivetrain drivetrain, Claw claw, Arm arm) {
        
        return Commands.sequence(
            arm.armExtend(),
            claw.clawOpen(),
            arm.armStow(),
            new TurnToAngleTank(drivetrain, 180),
            new DriveStraight(drivetrain, 0.5, 0.762, 0, false),
            new TurnToAngleTank(drivetrain, -90),
            new DriveStraight(drivetrain, 0.5, 0.4064, 0, false),
            new TurnToAngleTank(drivetrain, 90),
            new DriveStraight(drivetrain, 0.5, 5.585, 0, false),
            claw.clawClose(),
            new TurnToAngleTank(drivetrain, 180),
            new DriveStraight(drivetrain, 0.5, 5.585, 0, false),
            new TurnToAngleTank(drivetrain, 90),
            new DriveStraight(drivetrain, 0.5, 0.4064, 0, false),
            new TurnToAngleTank(drivetrain, -90),
            new DriveStraight(drivetrain, 0.5, 0.762, 0, false),
            arm.armExtend(),
            claw.clawOpen(),
            arm.armStow(),            
            new TurnToAngleTank(drivetrain, 180),
            new DriveStraight(drivetrain, 0.5, 0.762, 0, false),
            new TurnToAngleTank(drivetrain, -90),
            new DriveStraight(drivetrain, 0.5, 0.6, 0, false),
            new TurnToAngleTank(drivetrain, 90),
            new DriveStraight(drivetrain, 0.5, 1.238, 0, false)
            );
        
    }

    public static CommandBase OverpricedVendingMachineAuto(TankDrivetrain drivetrain, Claw claw, Arm arm) {
        return Commands.sequence(
            arm.armExtend(),
            claw.clawOpen(),
            arm.armStow(),
            new TurnToAngleTank(drivetrain, 180),
            new DriveStraight(drivetrain, 0.5, 6.347, 0, false),
            claw.clawClose(),
            new TurnToAngleTank(drivetrain, 180),
            new DriveStraight(drivetrain, 0.5, 6.347, 0, false),
            arm.armExtend(),
            claw.clawOpen(),
            arm.armStow(),
            new TurnToAngleTank(drivetrain, 180),
            new DriveStraight(drivetrain, 0.5, 1.238, 0, false)
        );
    }


    public static CommandBase chooseAuto(TankDrivetrain drivetrain, Claw claw, Arm arm, String location, boolean getGamePiece, boolean balanceRamp, boolean outCommunity) {
        if (location.equals("left")) {
            if (getGamePiece) {
                if (balanceRamp) {
                    return Commands.sequence(
                        HardCarryAuto(drivetrain, claw, arm),
                        new TheGreatBalancingTank(drivetrain)
                    );
                } else {
                    return HardCarryAuto(drivetrain, claw, arm);
                }
            } else {
                if (balanceRamp) {
                    return Commands.sequence(
                        arm.armExtend(),
                        claw.clawOpen(),
                        arm.armStow(),
                        new DriveStraight(drivetrain, -0.5, -0.762, 0, false),
                        new TurnToAngleTank(drivetrain, -90),
                        new DriveStraight(drivetrain, 0.5, 0.6, 0, false),
                        new TurnToAngleTank(drivetrain, -90),
                        new DriveStraight(drivetrain, 0.5, 1.238, 0, false),
                        new TheGreatBalancingTank(drivetrain)         
                    );
                } else {
                    return Commands.sequence(
                        arm.armExtend(),
                        claw.clawOpen(),
                        arm.armStow(),
                        new DriveStraight(drivetrain, -0.5, -0.762, 0, false),
                        new TurnToAngleTank(drivetrain, -90),
                        new DriveStraight(drivetrain, 0.5, 0.6, 0, false),
                        new TurnToAngleTank(drivetrain, -90),
                        new DriveStraight(drivetrain, 0.5, 1.238, 0, false)            
                    );
                }
            }
        } else if (location.equals("right")) {
            if (getGamePiece) {
                if (balanceRamp) {
                    return Commands.sequence(
                      DietCokeAuto(drivetrain, claw, arm),
                      new TheGreatBalancingTank(drivetrain)  
                    );
                } else {
                    return DietCokeAuto(drivetrain, claw, arm);
                }
            } else {
                if (balanceRamp) {
                    return Commands.sequence(
                        arm.armExtend(),
                        claw.clawOpen(),
                        arm.armStow(),            
                        new TurnToAngleTank(drivetrain, 180),
                        new DriveStraight(drivetrain, 0.5, 0.762, 0, false),
                        new TurnToAngleTank(drivetrain, -90),
                        new DriveStraight(drivetrain, 0.5, 0.6, 0, false),
                        new TurnToAngleTank(drivetrain, 90),
                        new DriveStraight(drivetrain, 0.5, 1.238, 0, false),
                        new TheGreatBalancingTank(drivetrain)            
                    );
                } else {
                    return Commands.sequence(
                        arm.armExtend(),
                        claw.clawOpen(),
                        arm.armStow(),            
                        new TurnToAngleTank(drivetrain, 180),
                        new DriveStraight(drivetrain, 0.5, 0.762, 0, false),
                        new TurnToAngleTank(drivetrain, -90),
                        new DriveStraight(drivetrain, 0.5, 0.6, 0, false),
                        new TurnToAngleTank(drivetrain, 90),
                        new DriveStraight(drivetrain, 0.5, 1.238, 0, false)         
                    );
                }
            }
        } else {
            if (getGamePiece) {
                if (balanceRamp) {
                    return Commands.sequence(
                        OverpricedVendingMachineAuto(drivetrain, claw, arm),
                        new TheGreatBalancingTank(drivetrain)
                    );
                } else {
                    return OverpricedVendingMachineAuto(drivetrain, claw, arm);
                }
            } else {
                if (balanceRamp) {
                    return Commands.sequence(
                        arm.armExtend(),
                        claw.clawOpen(),
                        arm.armStow(),
                        new TurnToAngleTank(drivetrain, 180),
                        new DriveStraight(drivetrain, 0.5, 1.238, 0, false),
                        new TheGreatBalancingTank(drivetrain)            
                    );
                } else {
                    return Commands.sequence(
                        arm.armExtend(),
                        claw.clawOpen(),
                        arm.armStow(),
                        new TurnToAngleTank(drivetrain, 180),
                        new DriveStraight(drivetrain, 0.5, 1.238, 0, false)            
                    );
                }
            }
        }
    }


}
