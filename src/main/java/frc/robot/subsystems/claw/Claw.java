package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;

public interface Claw {
    public CommandBase intakeCone();
    public CommandBase intakeCube();
    public CommandBase outtakeCone();
    public CommandBase outtakeCube();

}
