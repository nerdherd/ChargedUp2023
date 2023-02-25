// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.subsystems.Reportable;

public class PistonClaw extends SubsystemBase implements Reportable, Claw {
    public DoubleSolenoid clawPiston;
    public boolean clawOpen;

    /**
     * Construct a new Claw subsystem.
     */
    public PistonClaw() {
        clawPiston = new DoubleSolenoid(PneumaticsConstants.kPCMPort, PneumaticsModuleType.CTREPCM, 
            ClawConstants.kPistonForwardID, ClawConstants.kPistonReverseID);
    }

    /**
     * Return an Instant Command that opens the claw.
     * @return  An Instant Command that opens the claw.
     */
    public CommandBase clawOpen() {
        return runOnce(
            () -> {
                clawPiston.set(Value.kForward);
                clawOpen = true;
            });
    }

    /**
     * Return an Instant Command that closes the claw.
     * @return  An Instant Command that closes the claw.
     */
    public CommandBase clawClose() {
        return runOnce(
            () -> {
                clawPiston.set(Value.kReverse);
                clawOpen = false;
            });
    }

    /**
     * Return an Instant Command that toggles the claw.
     * @return  An Instant Command that toggles the claw.
     */
    public CommandBase toggleClaw() {
        return runOnce(
            () -> {
                clawOpen = !clawOpen;
                if (clawOpen) {
                    clawOpen();
                } else {
                    clawClose();
                }
            });
    }

    /**
     * Query the value of clawOpen boolean.
     *
     * @return true if claw is open, false if claw is closed.
     */
    public boolean isClawOpen() {
        return clawOpen;
    }

    private CommandBase intake() {
        return Commands.sequence(
            clawOpen(),
            Commands.waitSeconds(0.5),
            clawClose()
        );
    }

    public CommandBase intakeCone() {return intake();}
    public CommandBase intakeCube() {return intake();}
    public CommandBase outtakeCone() {return intake();}
    public CommandBase outtakeCube() {return intake();}


    public void reportToSmartDashboard() {

    }

    public void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Claw");

        tab.addBoolean("Claw Open", this::isClawOpen);
    }
}
