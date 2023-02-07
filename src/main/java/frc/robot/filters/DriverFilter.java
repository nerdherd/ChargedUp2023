package frc.robot.filters;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriverFilter extends FilterSeries {
    public DriverFilter(double deadband, double alpha, 
            double oneMinusAlpha, double posRateLimit, 
            int power, double negRateLimit) {
        super(
            new DeadbandFilter(deadband),
            new LowPassFilter(alpha, oneMinusAlpha),
            new PowerFilter(power),
            new WrapperFilter(
                (x) -> {
                    SmartDashboard.putNumber("Output", x);
                    return x;
                }
            ),
            new WrapperFilter(
                new SlewRateLimiter(posRateLimit, negRateLimit, 0)::calculate
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
