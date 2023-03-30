// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.imu;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Reportable;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class NavX extends SubsystemBase implements Gyro {
    public AHRS ahrs;
    private int numResets = 0;
    private double offset = 0;
    
    /**
     * Attempt to instantiate a new NavX IMU.
     * 
     * If an exception is thrown, it is caught and reported to the drivetrain.
     */
    public NavX() {
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
        offset = 0;
        numResets += 1;
    }
    
    public void setOffset(double offset) {
        this.offset = offset;
    }

    public void resetHeading(double headingDegrees) {
        zeroHeading();
        offset = -headingDegrees;
    }

    /**
     * Gets angle robot is facing
     * @return Angle of the robot (degrees)
     */
    public double getHeading() {
        double heading = Math.IEEEremainder(ahrs.getYaw() - offset, 360);
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
            Math.toRadians(ahrs.getRoll()),
            Math.toRadians(ahrs.getPitch()),
            Math.toRadians(getHeading()));
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
            SmartDashboard.putNumber("IMU Resets", numResets);
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
                tab.addNumber("Robot Raw X Acceleration", () -> ahrs.getRawAccelX());
                tab.addNumber("Robot Raw Y Acceleration", () -> ahrs.getRawAccelY());
                tab.addNumber("Robot Raw Z Acceleration", () -> ahrs.getRawAccelZ());
                tab.addBoolean("AHRS Calibrating", () -> ahrs.isCalibrating());
                tab.addBoolean("AHRS Connected", () -> ahrs.isConnected());
                tab.addString("NavX Firmware version", () -> ahrs.getFirmwareVersion());
                tab.addNumber("Gyro Temperature (C)", () -> ahrs.getTempC());
            case MEDIUM:
                tab.addNumber("Gyro Yaw", () -> ahrs.getYaw());
                tab.addNumber("Gyro Pitch", () -> ahrs.getPitch());
                tab.addNumber("Gyro Roll", () -> ahrs.getRoll());
                tab.addNumber("Gyro X Acceleration (G)", () -> ahrs.getWorldLinearAccelX());
                tab.addNumber("Gyro Y Acceleration (G)", () -> ahrs.getWorldLinearAccelY());
                tab.addNumber("Gyro Z Acceleration (G)", () -> ahrs.getWorldLinearAccelZ());
                tab.addNumber("Total Acceleration (m/s2)", () -> {
                    return SwerveDriveConstants.kGravityMPS
                        * (ahrs.getWorldLinearAccelX()
                        + ahrs.getWorldLinearAccelY()
                        + ahrs.getWorldLinearAccelZ());
                });
                tab.addNumber("Gyro X Displacement (m)", () -> ahrs.getDisplacementX());
                tab.addNumber("Gyro Y Displacement (m)", () -> ahrs.getDisplacementY());
                tab.addNumber("Gyro Z Displacement (m)", () -> ahrs.getDisplacementZ());
                tab.addNumber("Gyro Full Range Acceleration (G)", () -> ahrs.getAccelFullScaleRangeG());
                tab.add("Reset Gyro", new InstantCommand(() -> ahrs.reset()));
            case MINIMAL:
                tab.addNumber("IMU Resets", () -> numResets);
                tab.addNumber("Robot Heading", () -> getHeading());
                break;
        }
    }
}