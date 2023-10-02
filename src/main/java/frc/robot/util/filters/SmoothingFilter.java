package frc.robot.util.filters;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class SmoothingFilter extends FilterSeries {
    private SlewRateLimiter slewRateLimiter;
    public SmoothingFilter(double alpha, double oneMinusAlpha,
        double posRateLimit, double negRateLimit) {
        this.slewRateLimiter = new SlewRateLimiter(posRateLimit, negRateLimit, 0);

        super.setFilters(
            new ExponentialSmoothingFilter(alpha, oneMinusAlpha),
            new WrapperFilter(
                (x) -> slewRateLimiter.calculate(x)
            )
        );
    }
}
