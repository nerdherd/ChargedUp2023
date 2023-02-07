package frc.robot.tests;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.filters.CubicFilter;
import frc.robot.filters.DriverFilter;
import frc.robot.filters.WrapperFilter;

/**
 * Class for running tests on the filtering library.
 */
public class FilterTest {
    public CubicFilter filter = new CubicFilter();
    public DriverFilter filter2 = new DriverFilter(
        OIConstants.kDeadband, 
        SwerveDriveConstants.kDriveAlpha, 
        SwerveDriveConstants.kDriveOneMinusAlpha,
        SwerveDriveConstants.kTeleMaxAcceleration,
        3,
        -SwerveDriveConstants.kTeleMaxAcceleration);
    public SlewRateLimiter yLimiter = new SlewRateLimiter(3);
    public double prevYInput = 0;

    public SlewRateLimiter limiter = new SlewRateLimiter(3);
    public SlewRateLimiter limiter2 = new SlewRateLimiter(3);

    public WrapperFilter slewFilter = new WrapperFilter(
        (x) -> {
            return limiter.calculate(x);
        }
    );
    
    public FilterTest() {}

    public void initialize() {
        SmartDashboard.putNumber("Input", 0);
    }

    public void periodic() {
        SmartDashboard.putNumber("Cubic Filter output: ", filter.calculate(SmartDashboard.getNumber("Input", 0)));
        
        // New version of swerve drive filter
        SmartDashboard.putNumber("New Filter: ", filter2.calculate(SmartDashboard.getNumber("Input", 0)));
        SmartDashboard.putNumber("Slew Filter Wrapped output", slewFilter.calculate(SmartDashboard.getNumber("Input", 0)));
        SmartDashboard.putNumber("Slew Filter output", limiter2.calculate(SmartDashboard.getNumber("Input", 0)));
        

        // Copy of original swerve drive filter

        double originalFilterValue = SmartDashboard.getNumber("Input", 0);

        // Apply deadband to the speeds
        originalFilterValue = (Math.abs(originalFilterValue) > OIConstants.kDeadband) ? originalFilterValue : 0; 

        // Apply low pass filter
        originalFilterValue = (SwerveDriveConstants.kDriveAlpha * originalFilterValue) + (SwerveDriveConstants.kDriveOneMinusAlpha * prevYInput);

        prevYInput = originalFilterValue;

        // Apply cubic
        originalFilterValue = Math.signum(originalFilterValue) * Math.abs(originalFilterValue * originalFilterValue * originalFilterValue);

        // Apply the slew rate limiter to the speeds
        originalFilterValue = yLimiter.calculate(originalFilterValue);

        SmartDashboard.putNumber("Original filter output:", originalFilterValue);
    }
}
