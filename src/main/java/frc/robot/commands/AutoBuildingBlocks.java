package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.claw.MotorClaw;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class AutoBuildingBlocks {
    public enum AutoBuildingBlock {
        OUTTAKE, INTAKE, EXTENDHIGH
    }

    private static HashMap<AutoBuildingBlock, Command> commandMap = new HashMap<>();

    public static void init(SwerveDrivetrain swerve, Arm arm, Elevator elevator, MotorClaw claw) { // add more subsystems
        commandMap.put(AutoBuildingBlock.OUTTAKE, Commands.sequence(
            claw.outtake() 
        ));

        commandMap.put(AutoBuildingBlock.INTAKE, Commands.sequence(
            claw.intake() 
        )); 
        commandMap.put(AutoBuildingBlock.EXTENDHIGH, Commands.sequence(
        
        ));

    }

    public static Command getCommand(AutoBuildingBlock commandType) {
        Command command = commandMap.get(commandType);
        if (command == null) {
            DriverStation.reportWarning("Auto command map has not been initialized!", true);
        }
        return commandMap.get(commandType);    
    }
    
}
