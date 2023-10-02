package frc.robot.filters;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.filters.ClampFilter;
import frc.robot.util.filters.DeadbandFilter;
import frc.robot.util.filters.ExponentialSmoothingFilter;
import frc.robot.util.filters.FilterSeries;
import frc.robot.util.filters.PowerFilter;
import frc.robot.util.filters.ScaleFilter;
import frc.robot.util.filters.WrapperFilter;

/**
 * New driver filter for swerve drive teleop input. Accepts a value in [-1, 1]
 */
public class OldDriverFilter2 extends FilterSeries {
    private SlewRateLimiter slewRateLimiter;
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
    public OldDriverFilter2(double deadband, double motorDeadband, double scale, 
            double alpha, double posRateLimit, double negRateLimit) {
        super();

        deadbandScaler = (1 - motorDeadband) * (1 - motorDeadband);

        if (negRateLimit > -3) {
            DriverStation.reportWarning(
                String.format(
                    "Swerve deceleration Value of %f is too low!", 
                    negRateLimit), 
                true);
            return;
        }

        slewRateLimiter = new SlewRateLimiter(posRateLimit, negRateLimit, 0);

        super.setFilters(
            new DeadbandFilter(deadband),
            new ScaleFilter(1.7),
            new ClampFilter(1),
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
            new WrapperFilter(
                (x) -> {
                    return Math.signum(x) 
                        * slewRateLimiter.calculate(Math.abs(x));
                }
            ),
            new ScaleFilter(scale),
            new ClampFilter(scale)
        );
    }
}
