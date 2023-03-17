package frc.robot.filters;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * New driver filter for swerve drive teleop input. Accepts a value in [-1, 1]
 */
public class NewDriverFilter extends FilterSeries {
    private boolean belowDeadband;
    private final double deadbandScaler;

    /**
     * Construct a new driver filter
     * @param deadband
     * @param motorDeadband
     * @param scale
     * @param alpha
     * @param posRateLimit
     * @param negRateLimit
     */
    public NewDriverFilter(double deadband, double motorDeadband, double scale, double alpha) {
        super();

        deadbandScaler = (1 - motorDeadband) * (1 - motorDeadband);
        super.setFilters(
            new DeadbandFilter(deadband),
            new ReverseDeadbandFilter(deadband, 1, -1),
            new WrapperFilter(
                (x) -> {
                    if (x == 0) {
                        belowDeadband = true;
                        return 0.0;
                    } 
                    if (x > 0) {
                        belowDeadband = false;
                        return x - deadband;
                    } else {
                        belowDeadband = false;
                        return x + deadband;
                    }
                }
            ),
            new ExponentialSmoothingFilter(alpha),
            new PowerFilter(3),
            new WrapperFilter(
                (x) -> {
                    if (belowDeadband) return 0.0;
                    else {
                        return Math.signum(x) * ((Math.abs(x) / deadbandScaler) + motorDeadband);
                    }
                }
            ),
            new ScaleFilter(scale),
            new ClampFilter(scale)
        );
    }
}
