package frc.robot.subsystems.imu;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pigeon extends SubsystemBase implements Gyro {
    private PigeonIMU pigeon;

    public Pigeon(int id) {
        try {
            this.pigeon = new PigeonIMU(id);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating Pigeon over CAN: " + ex.getMessage(), true);
        }
    }

    public Pigeon(TalonSRX srx) {
        try {
            this.pigeon = new PigeonIMU(srx);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating Pigeon connected to TalonSRX: " + ex.getMessage(), true);
        }
    }
    
    public void zeroHeading() {
        pigeon.setYaw(0);
    }

    public double getHeading() {
        return -pigeon.getCompassHeading() % 360;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
    
    /**
     * For orientations, see page 20 of {@link https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User%27s%20Guide.pdf}
     */
    public Rotation3d getRotation3d() {
        return new Rotation3d(
            Math.toRadians(-pigeon.getRoll() % 360),
            Math.toRadians(pigeon.getPitch() % 360),
            Math.toRadians(-pigeon.getYaw() % 360)
        );
    }
    
    public void reportToSmartDashboard() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Robot Yaw", -pigeon.getYaw());
        SmartDashboard.putNumber("Robot Pitch", pigeon.getPitch());
        SmartDashboard.putNumber("Robot Roll", -pigeon.getRoll());
        SmartDashboard.putNumber("Pigeon Firmware Version", pigeon.getFirmwareVersion());
    }
    
    public void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Imu");

        tab.addNumber("Robot Heading", this::getHeading);
        tab.addNumber("Robot Yaw", () -> -pigeon.getYaw());
        tab.addNumber("Robot Pitch", () -> pigeon.getPitch());
        tab.addNumber("Robot Roll", () -> -pigeon.getRoll());
        tab.addNumber("Pigeon Firmware Version", () -> pigeon.getFirmwareVersion());
    }
    
}
