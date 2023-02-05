package frc.robot.filters;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class DriverFilter extends FilterSeries {
    public DriverFilter(double deadband, double alpha, 
            double oneMinusAlpha, double posRateLimit, 
            int power, double negRateLimit) {
        super(
            new DeadbandFilter(deadband),
            new LowPassFilter(alpha, oneMinusAlpha),
            new PowerFilter(power),
            new WrapperFilter(
                new SlewRateLimiter(posRateLimit, negRateLimit, 0)::calculate
            )
        );
    }
}
