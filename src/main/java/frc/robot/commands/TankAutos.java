package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

public class TankAutos {
    
    public static CommandBase HardCarryAuto(Drivetrain drivetrain, Claw claw, Arm arm) {
        return Commands.sequence(
            // arm.armExtend(),
            claw.clawOpen(),
            // arm.armStow(),
            // new TurnToAngleTank(drivetrain, 180),
            // new DriveStraight(drivetrain, 0.5, 0.762, 0, false),
            // new TurnToAngleTank(drivetrain, 90),
            // new DriveStraight(drivetrain, 0.5, 0.4064, 0, false),
            // new TurnToAngleTank(drivetrain, -90),
            // new DriveStraight(drivetrain, 0.5, 5.585, 0, false),
            // claw.clawClose(),
            // new TurnToAngleTank(drivetrain, 180),
            // new DriveStraight(drivetrain, 0.5, 5.585, 0, false),
            // new TurnToAngleTank(drivetrain, -90),
            // new DriveStraight(drivetrain, 0.5, 0.4064, 0, false),
            // new TurnToAngleTank(drivetrain, 90),
            // new DriveStraight(drivetrain, 0.5, 0.762, 0, false),
            // arm.armExtend(),
            // claw.clawOpen(),
            // arm.armStow(),

            // 
            // new DriveStraight(drivetrain, -0.5, -0.762, 0, false)
            // new DriveStraight(drivetrain, 0.5, 2.762, 0, false),
            new DriveStraight(drivetrain, 0.5, -2.762, 0, false),

            new TurnToAngleTank(drivetrain, 90)
            // new DriveStraight(drivetrain, 0.5, 0.762, 0, false),
            // new TurnToAngleTank(drivetrain, 90),
            // new DriveStraight(drivetrain, 0.5, 0.6, 0, false),
            // new TurnToAngleTank(drivetrain, -90),
            // new DriveStraight(drivetrain, 0.5, 1.238, 0, false),
            // new TheGreatBalancingTank(drivetrain)
            );
    }

    public static CommandBase DietCokeAuto(Drivetrain drivetrain, Claw claw, Arm arm) {
        
        return Commands.sequence(
            // arm.armExtend(),
            claw.clawOpen(),
            // arm.armStow(),
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
            // arm.armExtend(),
            claw.clawOpen(),
            // arm.armStow(),            
            new TurnToAngleTank(drivetrain, 180),
            new DriveStraight(drivetrain, 0.5, 0.762, 0, false),
            new TurnToAngleTank(drivetrain, -90),
            new DriveStraight(drivetrain, 0.5, 0.6, 0, false),
            new TurnToAngleTank(drivetrain, 90),
            new DriveStraight(drivetrain, 0.5, 1.238, 0, false),
            new TheGreatBalancingTank(drivetrain)
            );
        
    }

    public static CommandBase OverpricedVendingMachineAuto(Drivetrain drivetrain, Claw claw, Arm arm) {
        return Commands.sequence(
            // arm.armExtend(),
            claw.clawOpen(),
            // arm.armStow(),
            new TurnToAngleTank(drivetrain, 180),
            new DriveStraight(drivetrain, 0.5, 6.347, 0, false),
            claw.clawClose(),
            new TurnToAngleTank(drivetrain, 180),
            new DriveStraight(drivetrain, 0.5, 6.347, 0, false),
            // arm.armExtend(),
            claw.clawOpen(),
            // arm.armStow(),
            new TurnToAngleTank(drivetrain, 180),
            new DriveStraight(drivetrain, 0.5, 1.238, 0, false),
            new TheGreatBalancingTank(drivetrain)
        );
    }


}
