package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;

public class Superstructure {

    private Arm arm;
    private Elevator elevator;
    private MotorClaw claw;

    public Superstructure(Arm arm, Elevator elevator, MotorClaw claw) {
        this.arm = new Arm();
        this.elevator = new Elevator();
        this.claw = new MotorClaw();
    }

}