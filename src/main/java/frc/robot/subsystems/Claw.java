// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    public DoubleSolenoid clawPiston;
    public boolean clawOpen;

    public Claw() {
        clawPiston = new DoubleSolenoid(ClawConstants.kPCMPort, PneumaticsModuleType.CTREPCM, ClawConstants.kPistonForwardID, ClawConstants.kPistonReverseID);
    }

    public CommandBase clawOpen() {
        return runOnce(
            () -> {
                clawPiston.set(Value.kForward);
                clawOpen = true;
            });
    }

    public CommandBase clawClose() {
        return runOnce(
            () -> {
                clawPiston.set(Value.kReverse);
                clawOpen = false;
            });
    }

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
}
