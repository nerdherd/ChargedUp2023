package frc.robot.subsystems.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.Reportable;

/**
 * Gyro interface for interoperability between Pigeon and NavX.
 * <p>
 * Does NOT include accelerometer functionality.
 * <p>
 * Follows CCW = positive conventions.
 * See page 20 of {@link https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User%27s%20Guide.pdf}
 * for Pigeon 2 orientations and see 
 * {@link https://pdocs.kauailabs.com/navx-mxp/installation/orientation-2/}
 * for NavX orientations.
 * <p>
 */
public interface Gyro extends Reportable {
    /** Soft reset all axes */
    public void zeroAll();
    public void zeroHeading();
    /** Soft reset the pitch */
    public void zeroPitch();
    /** Soft reset the roll */
    public void zeroRoll();
    /** Soft reset the heading */
    public void resetHeading(double headingDegrees);
    /** Soft reset the pitch */
    public void resetPitch(double pitchDegrees);
    /** Soft reset the roll */
    public void resetRoll(double rollDegrees);
    /** Set the yaw offset */
    public void setOffset(double offset);
    /** Set the pitch offset */
    public void setPitchOffset(double offset);
    /** Set the roll offset */
    public void setRollOffset(double offset);
    /**
     * Get the heading in degrees. (CCW = positive)
     * @return  IMU Heading in degrees.
     */
    public double getHeading();
    /**
     * Get the pitch in degrees. (CCW = positive)
     * @return  IMU Heading in degrees.
     */
    public double getPitch();
    /**
     * Get the roll in degrees. (CCW = positive)
     * @return  IMU Heading in degrees.
     */
    public double getRoll();
    public double getHeadingOffset();
    public double getRollOffset();
    public double getPitchOffset();
    public Rotation2d getRotation2d();
    public Rotation3d getRotation3d();
}
