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
 * The original driver filter used to filter swerve drive teleop input.
 * Accepts a value in [-1, 1]
 */
public class OldDriverFilter extends FilterSeries {
    private SlewRateLimiter slewRateLimiter;

    public OldDriverFilter(double deadband, double alpha, 
            double oneMinusAlpha, double maxSpeed, 
            double posRateLimit, int power, 
            double negRateLimit) {
        super();

        if (negRateLimit > -3) {
            DriverStation.reportWarning(
                String.format(
                    "Swerve deceleration Value of %d is too low!", 
                    negRateLimit), 
                true);
            return;
        }

        slewRateLimiter = new SlewRateLimiter(posRateLimit, negRateLimit, 0);
        
        super.setFilters(
            new DeadbandFilter(deadband),
            new ExponentialSmoothingFilter(alpha, oneMinusAlpha),
            new PowerFilter(power),
            new WrapperFilter(
                (x) -> {
                    return slewRateLimiter.calculate(x);
                }
            ),
            new ScaleFilter(maxSpeed),
            new ClampFilter(maxSpeed, -maxSpeed)
        );
    }

    public OldDriverFilter(double deadband, double alpha, 
            double oneMinusAlpha, double maxSpeed, double rateLimit, int power) {
        this(deadband, alpha, oneMinusAlpha, maxSpeed, rateLimit, power, -rateLimit);
    }

    public OldDriverFilter(double deadband, double alpha, 
        double oneMinusAlpha, double maxSpeed, double rateLimit) {
        this(deadband, alpha, oneMinusAlpha, maxSpeed, rateLimit, 3, -rateLimit);
    }
}
