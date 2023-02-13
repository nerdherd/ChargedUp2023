package frc.robot.filters;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The original driver filter used to filter swerve drive teleop input.
 * Accepts a value in [-1, 1]
 */
public class DriverFilter extends FilterSeries {
    private SlewRateLimiter slewRateLimiter;

    public DriverFilter(double deadband, double alpha, 
            double oneMinusAlpha, double maxSpeed, 
            double posRateLimit, int power, 
            double negRateLimit) {
        super();

        slewRateLimiter = new SlewRateLimiter(posRateLimit, negRateLimit, 0);
        
        super.setFilters(
            new DeadbandFilter(deadband),
            new ExponentialSmoothingFilter(alpha, oneMinusAlpha),
            new PowerFilter(power),
            new WrapperFilter(
                (x) -> {
                    return Math.signum(x) 
                        * slewRateLimiter.calculate(Math.abs(x));
                }
            ),
            new ScaleFilter(maxSpeed),
            new ClampFilter(maxSpeed, -maxSpeed)
        );
    }

    public DriverFilter(double deadband, double alpha, 
            double oneMinusAlpha, double maxSpeed, double rateLimit, int power) {
        this(deadband, alpha, oneMinusAlpha, maxSpeed, rateLimit, power, -rateLimit);
    }

    public DriverFilter(double deadband, double alpha, 
        double oneMinusAlpha, double maxSpeed, double rateLimit) {
        this(deadband, alpha, oneMinusAlpha, maxSpeed, rateLimit, 3, -rateLimit);
    }
}
