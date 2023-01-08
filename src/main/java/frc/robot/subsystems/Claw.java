// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    public DoubleSolenoid clawPiston;
    public boolean clawOpen;

  /** Creates a new ExampleSubsystem. */
    public Claw() {
        clawPiston = new DoubleSolenoid(ClawConstants.kPCMPort, PneumaticsModuleType.CTREPCM, ClawConstants.kPistonForwardID, ClawConstants.kPistonReverseID);
    }

    public void clawOpen() {
        clawPiston.set(Value.kForward);
        clawOpen = true;
    }
    
    public void clawClose() {
        clawPiston.set(Value.kReverse);
        clawOpen = false;
    }
    
    public void toggleClaw() {
        clawOpen = !clawOpen;
        if (clawOpen) {
          clawOpen();
        } else {
          clawClose();
        }
    }

}
