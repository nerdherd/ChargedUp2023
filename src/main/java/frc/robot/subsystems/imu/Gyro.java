package frc.robot.subsystems.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.Reportable;

/**
 * Gyro interface for interoperability between Pigeon and NavX.
 * <p>
 * Does NOT include accelerometer functionality.
 * <p>
 * Follows CW = positive conventions.
 * See page 20 of {@link https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User%27s%20Guide.pdf}
 * for Pigeon 2 orientations and see 
 * {@link https://pdocs.kauailabs.com/navx-mxp/installation/orientation-2/}
 * for NavX orientations.
 * <p>
 */
//TODO: Change to CCW = positive to match with WPILib conventions
public interface Gyro extends Reportable {
    public void zeroHeading();
    /**
     * Get the heading in degrees. (CW = positive)
     * @return  IMU Heading in degrees.
     */
    public double getHeading();
    public Rotation2d getRotation2d();
    public Rotation3d getRotation3d();
}
