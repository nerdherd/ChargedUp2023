// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import edu.wpi.first.wpilibj.SPI;

public class Imu extends SubsystemBase {
    public AHRS ahrs;
    // add compressor
    
    public Imu() {
        try { // attempt to instantiate the NavX2. If it throws an exception, catch it and
            // report it.
          ahrs = new AHRS(SPI.Port.kMXP); // SPI is the protocol on the MXP connector that the navigator is plugged
                                          // into
      } catch (RuntimeException ex) {
          DriverStation.reportError("Error instantiating navX2 MXP:  " + ex.getMessage(), true);
      }
    }
}
