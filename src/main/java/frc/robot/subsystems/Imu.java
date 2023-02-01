// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

// TODO: Wrap crucial ahrs methods so that functionality is swappable between the NavX and Pigeon IMU

public class Imu extends SubsystemBase {
    public AHRS ahrs;
    
    /**
     * Attempt to instantiate a new NavX IMU.
     * 
     * If an exception is thrown, it is caught and reported to the drivetrain.
     */
    public Imu() {
        try { 
            // SPI is the protocol on the MXP connector that 
            // the NavX is plugged into
            ahrs = new AHRS(SPI.Port.kMXP); 
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX2 MXP:  " + ex.getMessage(), true);
        }
    }
}
