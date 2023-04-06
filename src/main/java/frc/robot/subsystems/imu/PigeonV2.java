package frc.robot.subsystems.imu;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PigeonV2 extends SubsystemBase implements Gyro {
    private Pigeon2 pigeon;
    private double offset, pitchOffset, rollOffset = 0;

    public PigeonV2(int id) {
        try {
            this.pigeon = new Pigeon2(id);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating Pigeon 2 over CAN: " + ex.getMessage(), true);
        }
        offset = 0;
        pitchOffset = 0;
        rollOffset = 0;
    }

    public void zeroAll() {
        zeroHeading();
        zeroPitch();
        zeroRoll();
    }
    
    public void zeroHeading() {
        pigeon.setYaw(0);
        offset = 0;
    }

    public void zeroPitch() {
        this.pitchOffset = pigeon.getPitch();
    }
    
    public void zeroRoll() {
        this.rollOffset = -pigeon.getRoll();
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public void setPitchOffset(double offset) {
        this.pitchOffset = offset;
    }

    public void setRollOffset(double offset) {
        this.rollOffset = offset;
    }

    public void resetHeading(double headingDegrees) {
        zeroHeading();
        offset = -headingDegrees;
    }

    public void resetPitch(double pitchDegrees) {
        this.pitchOffset = pigeon.getPitch() - pitchDegrees;
    }

    public void resetRoll(double rollDegrees) {
        this.rollOffset = -pigeon.getRoll() - rollDegrees;
    }

    public double getHeading() {
        return (-pigeon.getCompassHeading() - offset) % 360;
    }

    public double getPitch() {
        return (pigeon.getPitch() - pitchOffset) % 360;
    }

    public double getRoll() {
        return (-pigeon.getRoll() - rollOffset) % 360;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
    
    /**
     * For orientations, see page 20 of {@link https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User%27s%20Guide.pdf}
     */
    public Rotation3d getRotation3d() {
        return new Rotation3d(
            Math.toRadians(getRoll()),
            Math.toRadians(getPitch()),
            Math.toRadians(getHeading())
        );
    }
    
    public void reportToSmartDashboard(LOG_LEVEL level) {
        switch (level) {
            case OFF:
                break;
            case ALL:
                SmartDashboard.putNumber("Pigeon Firmware Version", pigeon.getFirmwareVersion());
            case MEDIUM:
                SmartDashboard.putNumber("Robot Yaw", -pigeon.getYaw());
                SmartDashboard.putNumber("Robot Pitch", pigeon.getPitch());
                SmartDashboard.putNumber("Robot Roll", -pigeon.getRoll());
            case MINIMAL:
                SmartDashboard.putNumber("Robot Heading", getHeading());
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
                tab.addNumber("Pigeon Firmware Version", () -> pigeon.getFirmwareVersion());
            case MEDIUM:
                tab.addNumber("Robot Yaw", () -> -pigeon.getYaw());
                tab.addNumber("Robot Pitch", () -> pigeon.getPitch());
                tab.addNumber("Robot Roll", () -> -pigeon.getRoll());
            case MINIMAL:
                tab.addNumber("Robot Heading", this::getHeading);
        }
    }
    
}
