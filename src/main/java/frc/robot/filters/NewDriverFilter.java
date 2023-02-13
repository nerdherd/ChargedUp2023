package frc.robot.filters;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * New driver filter for swerve drive teleop input. Accepts a value in [-1, 1]
 */
public class NewDriverFilter extends FilterSeries {
    private SlewRateLimiter slewRateLimiter;
    private boolean belowDeadband;
    private final double deadbandScaler;

    public NewDriverFilter(double deadband, double scale, 
            double alpha, double posRateLimit, double negRateLimit) {
        super();

        slewRateLimiter = new SlewRateLimiter(posRateLimit, negRateLimit, 0);
        deadbandScaler = (1 - deadband) * (1 - deadband);

        super.setFilters(
            new DeadbandFilter(deadband),
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
                        return Math.signum(x) * ((Math.abs(x) / deadbandScaler) + deadband);
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
