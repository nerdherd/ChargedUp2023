package frc.robot.tests;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.filters.DriverFilter;
import frc.robot.filters.NewDriverFilter;
import frc.robot.util.BadPS4;

/**
 * Class for running tests on the filtering library.
 */
public class FilterTest {
    public DriverFilter oldXFilter = new DriverFilter(
        OIConstants.kDeadband, 
        SwerveDriveConstants.kDriveAlpha, 
        SwerveDriveConstants.kDriveOneMinusAlpha,
        SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
        SwerveDriveConstants.kTeleMaxAcceleration,
        3,
        -SwerveDriveConstants.kTeleMaxAcceleration);
    
    public DriverFilter oldYFilter = new DriverFilter(
        OIConstants.kDeadband, 
        SwerveDriveConstants.kDriveAlpha, 
        SwerveDriveConstants.kDriveOneMinusAlpha,
        SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
        SwerveDriveConstants.kTeleMaxAcceleration,
        3,
        -SwerveDriveConstants.kTeleMaxAcceleration);
    
    public DriverFilter oldRotationFilter = new DriverFilter(
        OIConstants.kDeadband, 
        SwerveDriveConstants.kDriveAlpha, 
        SwerveDriveConstants.kDriveOneMinusAlpha,
        SwerveDriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,
        SwerveDriveConstants.kTeleMaxAcceleration,
        3,
        -SwerveDriveConstants.kTeleMaxAcceleration);
    
    public SlewRateLimiter originalLimiter = new SlewRateLimiter(3);
    public double prevYInput = 0;

    public double currentXPos = 0;
    public double currentYPos = 0;
    public Rotation2d currentAngle = new Rotation2d(0);

    public NewDriverFilter newXFilter = new NewDriverFilter(0.05, 0.05, SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond, SwerveDriveConstants.kDriveAlpha, SwerveDriveConstants.kTeleMaxAcceleration, SwerveDriveConstants.kTeleMaxDeceleration);
    public NewDriverFilter newYFilter = new NewDriverFilter(0.05, 0.05, SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond, SwerveDriveConstants.kDriveAlpha, SwerveDriveConstants.kTeleMaxAcceleration, SwerveDriveConstants.kTeleMaxDeceleration);
    public NewDriverFilter newRotationFilter = new NewDriverFilter(0.05, 0.05, SwerveDriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond, SwerveDriveConstants.kDriveAlpha, SwerveDriveConstants.kTeleMaxAcceleration, SwerveDriveConstants.kTeleMaxDeceleration);    

    public BadPS4 badPS4 = new BadPS4(0);

    public Field2d field;

    public FilterTest() {}

    public void initialize() {
        SmartDashboard.putNumber("X Input", 0);
        SmartDashboard.putNumber("Y Input", 0);

        field = new Field2d();
    }

    public void periodic() {
        SmartDashboard.putData(field);
    }

    private void updatePositions(double xSpeed, double ySpeed, double rotationSpeed, double loopTime, double minimumInput) {
        if (xSpeed > minimumInput || xSpeed < -minimumInput) {
            this.currentXPos += loopTime*xSpeed;
        }
        if (ySpeed > minimumInput || ySpeed < -minimumInput) {
            this.currentYPos += loopTime*ySpeed;
        }
        this.currentAngle = new Rotation2d(currentAngle.getRadians() + rotationSpeed*loopTime);
        field.setRobotPose(new Pose2d(currentXPos, currentYPos, currentAngle));
        SmartDashboard.putNumber("X Speed", xSpeed);
        SmartDashboard.putNumber("Y Speed", ySpeed);
        SmartDashboard.putNumber("Rotation Speed", rotationSpeed);
        SmartDashboard.putNumber("X Pos", currentXPos);
        SmartDashboard.putNumber("Y Pos", currentYPos);
        SmartDashboard.putNumber("Rotation Degrees", currentAngle.getDegrees());
    }

    private void updatePositions(double xSpeed, double ySpeed, double rotationSpeed) {
        updatePositions(xSpeed, ySpeed, rotationSpeed, 0.02, 0.05);
    }

    public void testBothFilters() {
        if (badPS4.getL1Button()) {
            newDriverFilterTest();
        } else {
            oldDriverFilterTest();
        }
    }

    public void newDriverFilterTest() {
        double xInput = badPS4.getLeftX();
        // double xInput = SmartDashboard.getNumber("X Input", 0);
        double yInput = -badPS4.getLeftY();
        // double yInput = SmartDashboard.getNumber("Y Input", 0);
        double rotation = badPS4.getRightX();

        double xSpeed = newXFilter.calculate(xInput);
        double ySpeed = newYFilter.calculate(yInput);
        double rotationSpeed = newRotationFilter.calculate(rotation);

        updatePositions(xSpeed, ySpeed, rotationSpeed);
    }

    public void oldDriverFilterTest() {
        double xInput = badPS4.getLeftX();
        double yInput = -badPS4.getLeftY();
        double rotation = badPS4.getRightX();

        double xSpeed = oldXFilter.calculate(xInput);
        double ySpeed = oldYFilter.calculate(yInput);
        double rotationSpeed = oldRotationFilter.calculate(rotation);

        updatePositions(xSpeed, ySpeed, rotationSpeed);
    }

    public void driveFilterTestPeriodic() {
        // New version of swerve drive filter
        SmartDashboard.putNumber("New Filter: ", oldXFilter.calculate(SmartDashboard.getNumber("Input", 0)));
        
    
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
