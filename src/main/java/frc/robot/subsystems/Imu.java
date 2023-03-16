// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// TODO: Wrap crucial ahrs methods so that functionality is swappable between the NavX and Pigeon IMU

public class Imu extends SubsystemBase implements Reportable {
    public AHRS ahrs;
    private int numResets = 0;
    
    /**
     * Attempt to instantiate a new NavX IMU.
     * 
     * If an exception is thrown, it is caught and reported to the drivetrain.
     */
    public Imu() {
        this.numResets = 0;
        
        try { 
            // SPI is the protocol on the MXP connector that 
            // the NavX is plugged into
            ahrs = new AHRS(SPI.Port.kMXP); 
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX2 MXP:  " + ex.getMessage(), true);
        }
    }
    
    /**
     * Set the current gyro direction to north
     */
    public void zeroHeading() {
        ahrs.reset();
        numResets += 1;
        SmartDashboard.putNumber("Gyro resets", numResets);
    }

    /**
     * Gets angle robot is facing
     * @return Angle of the robot (degrees)
     */
    public double getHeading() {
        double heading = Math.IEEEremainder(ahrs.getYaw(), 360);
        SmartDashboard.putNumber("Heading degrees", heading);
        return heading;
    }

    /**
     * Gets a rotation2d representing rotation of the drivetrain
     * @return A rotation2d representing rotation of the drivetrain
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Rotation3d getRotation3d() {
        return new Rotation3d(
            ahrs.getRoll() * Math.PI / 180, 
            ahrs.getPitch()* Math.PI / 180, 
            ahrs.getYaw() * Math.PI / 180) ;
    }

    public Rotation3d getRotation3dRaw() {
        
        return new Rotation3d(
            Math.toRadians(ahrs.getRawGyroX()),
            Math.toRadians(ahrs.getRawGyroY()),
            Math.toRadians(ahrs.getRawGyroZ())
        );
    }

    public void reportToSmartDashboard(LOG_LEVEL level) {
        switch (level) {
        case OFF:
            break;
        case ALL:
            SmartDashboard.putNumber("Robot Yaw", ahrs.getYaw());
            SmartDashboard.putNumber("Robot Pitch", ahrs.getPitch());
            SmartDashboard.putNumber("Robot Roll", ahrs.getRoll());
            SmartDashboard.putNumber("Robot Raw Yaw", ahrs.getRawGyroZ());
            SmartDashboard.putNumber("Robot Raw Pitch", ahrs.getRawGyroX());
            SmartDashboard.putNumber("Robot Raw Roll", ahrs.getRawGyroY());
            SmartDashboard.putBoolean("AHRS Calibrating", ahrs.isCalibrating());
            SmartDashboard.putBoolean("AHRS Connected", ahrs.isConnected());
            SmartDashboard.putString("NavX Firmware version", ahrs.getFirmwareVersion());
        case MEDIUM:
        case MINIMAL:
            SmartDashboard.putNumber("Robot Heading", getHeading());
            break;
        }
    }

    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF)  {
            return;
        }
        ShuffleboardTab tab;
        if (level == LOG_LEVEL.MINIMAL) {
            tab = Shuffleboard.getTab("Main");
        } else {
            tab = Shuffleboard.getTab("Imu");
        }
        switch (level) {
            case OFF:
                break;
            case ALL:
                tab.add("Calibrate NavX", new InstantCommand(() -> ahrs.calibrate()));
                tab.addNumber("Robot Raw Yaw", () -> ahrs.getRawGyroZ());
                tab.addNumber("Robot Raw Pitch", () -> ahrs.getRawGyroX());
                tab.addNumber("Robot Raw Roll", () -> ahrs.getRawGyroY());
                tab.addBoolean("AHRS Calibrating", () -> ahrs.isCalibrating());
                tab.addBoolean("AHRS Connected", () -> ahrs.isConnected());
                tab.addString("NavX Firmware version", () -> ahrs.getFirmwareVersion());
            case MEDIUM:
                tab.addNumber("Robot Yaw", () -> ahrs.getYaw());
                tab.addNumber("Robot Pitch", () -> ahrs.getPitch());
                tab.addNumber("Robot Roll", () -> ahrs.getRoll());
                tab.add("Reset Gyro", new InstantCommand(() -> ahrs.reset()));
            case MINIMAL:
                tab.addNumber("Robot Heading", () -> getHeading());
                break;
        }
    }
}
