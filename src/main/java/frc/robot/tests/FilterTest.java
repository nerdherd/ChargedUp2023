package frc.robot.tests;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.filters.DriverFilter;
import frc.robot.filters.NewDriverFilter;

/**
 * Class for running tests on the filtering library.
 */
public class FilterTest {
    public DriverFilter driverFilter = new DriverFilter(
        OIConstants.kDeadband, 
        SwerveDriveConstants.kDriveAlpha, 
        SwerveDriveConstants.kDriveOneMinusAlpha,
        SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
        SwerveDriveConstants.kTeleMaxAcceleration,
        3,
        -SwerveDriveConstants.kTeleMaxAcceleration);
    public SlewRateLimiter originalLimiter = new SlewRateLimiter(3);
    public double prevYInput = 0;

    public FilterTest() {}

    public void initialize() {
        SmartDashboard.putNumber("Input", 0);
    }

    public void periodic() {
    }

    public void newDriveFilterTestPeriodic() {
        NewDriverFilter newDriveFilter = new NewDriverFilter(OIConstants.kDeadband);
        double input = SmartDashboard.getNumber("Input", 0);
        SmartDashboard.putNumber("New Driver Filter Output", newDriveFilter.calculate(input));
    }
    
    public void driveFilterTestPeriodic() {
        // New version of swerve drive filter
        SmartDashboard.putNumber("New Filter: ", driverFilter.calculate(SmartDashboard.getNumber("Input", 0)));
        
    
        // Copy of original swerve drive filter
        double originalFilterValue = SmartDashboard.getNumber("Input", 0);
    
        // Apply deadband to the speeds
        originalFilterValue = (Math.abs(originalFilterValue) > OIConstants.kDeadband) ? originalFilterValue : 0; 
    
        // Apply "low pass" filter
        originalFilterValue = (SwerveDriveConstants.kDriveAlpha * originalFilterValue) + (SwerveDriveConstants.kDriveOneMinusAlpha * prevYInput);
        prevYInput = originalFilterValue;
    
        // Apply cubic
        originalFilterValue = Math.signum(originalFilterValue) * Math.abs(originalFilterValue * originalFilterValue * originalFilterValue);
    
        SmartDashboard.putNumber("Original pre-slew:", originalFilterValue);
    
        // Apply the slew rate limiter to the speeds
        originalFilterValue = originalLimiter.calculate(originalFilterValue);
    
        // Apply scale
        originalFilterValue *= SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    
        SmartDashboard.putNumber("Original filter output:", originalFilterValue);

    }
}
