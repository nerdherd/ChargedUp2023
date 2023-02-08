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
            double oneMinusAlpha, double posRateLimit, 
            int power, double negRateLimit) {
        super();

        slewRateLimiter = new SlewRateLimiter(posRateLimit, negRateLimit, 0);
        
        super.setFilters(
            new DeadbandFilter(deadband),
            new ExponentialSmoothingFilter(alpha, oneMinusAlpha),
            new PowerFilter(power),
            new WrapperFilter(
                (x) -> {
                    SmartDashboard.putNumber("Pre-slew", x);
                    return slewRateLimiter.calculate(x);
                }
            )
        );
    }

    public DriverFilter(double deadband, double alpha, 
            double oneMinusAlpha, double posRateLimit, int power) {
        this(deadband, alpha, oneMinusAlpha, posRateLimit, power, -posRateLimit);
    }

    public DriverFilter(double deadband, double alpha, 
        double oneMinusAlpha, double posRateLimit) {
        this(deadband, alpha, oneMinusAlpha, posRateLimit, 3, -posRateLimit);
    }
}
