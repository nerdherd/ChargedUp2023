package frc.robot.filters;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.filters.ClampFilter;
import frc.robot.util.filters.DeadbandFilter;
import frc.robot.util.filters.FilterSeries;
import frc.robot.util.filters.PowerFilter;
import frc.robot.util.filters.ScaleFilter;
import frc.robot.util.filters.WrapperFilter;

/**
 * New driver filter for swerve drive teleop input. Accepts a value in [-1, 1]
 */
public class DriverFilter extends FilterSeries {
    private boolean belowDeadband;

    /**
     * Construct a new driver filter
     */
    public DriverFilter(double deadband, double motorDeadband, double scale) {
        super();

        super.setFilters(
            new DeadbandFilter(deadband),
            new WrapperFilter((x) -> {
                if (x == 0) { 
                    belowDeadband = true; 
                    return 0.0; 
                }
                return Math.signum(x) * (Math.abs(x) - deadband) / (1-deadband);
            }),
            new PowerFilter(3),
            new WrapperFilter((x) -> {
                if (belowDeadband) return x*(1-motorDeadband) + Math.signum(x)*motorDeadband;
                return x;
            }),
            new ScaleFilter(scale),
            new ClampFilter(scale)
        );
    }
}
