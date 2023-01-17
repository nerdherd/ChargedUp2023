package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.SwerveDriveConstants.*;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrivetrain;

public class SwerveJoystickCommand extends CommandBase {
    private final SwerveDrivetrain swerveDrive;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final Supplier<Boolean> towSupplier;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    private double prevXInput, prevYInput = 0.0;

    /**
     * Construct a new SwerveJoystickCommand
     * 
     * @param swerveDrive           The Swerve Drive subsystem
     * @param xSpdFunction          A supplier returning the desired x speed
     * @param ySpdFunction          A supplier returning the desired y speed
     * @param turningSpdFunction    A supplier returning the desired turning speed
     * @param fieldOrientedFunction A supplier returning whether or not the function is field oriented
     */
    public SwerveJoystickCommand(SwerveDrivetrain swerveDrive,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> towSupplier) {
        this.swerveDrive = swerveDrive;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.towSupplier = towSupplier;
        this.xLimiter = new SlewRateLimiter(kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveDrive);
    }
     
    @Override
    public void initialize() {}

    /**
     * Apply deadband to the input
     * @param input     The input to apply deadband to
     * @return
     */
    public double applyDeadband(double input) {
        return (Math.abs(input) > OIConstants.kDeadband) ? input : 0; 
    }

    @Override
    public void execute() {
        if (towSupplier.get()) {
            swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates);
            return;
        }

        // get speeds
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // Apply deadband to the speeds
        xSpeed = applyDeadband(xSpeed);
        ySpeed = applyDeadband(ySpeed);
        turningSpeed = applyDeadband(turningSpeed);

        // Apply low pass filter
        xSpeed = (kDriveAlpha * xSpeed) + (kDriveOneMinusAlpha * prevXInput);
        ySpeed = (kDriveAlpha * ySpeed) + (kDriveOneMinusAlpha * prevYInput);

        prevXInput = xSpeed;
        prevYInput = ySpeed;

        // Apply quadratic
        xSpeed = Math.signum(xSpeed) * xSpeed * xSpeed;
        ySpeed = Math.signum(ySpeed) * ySpeed * ySpeed;

        // Apply the slew rate limiter to the speeds
        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        turningSpeed = turningLimiter.calculate(turningSpeed);

        // Convert speeds to meters per second
        xSpeed *= kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed *= kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed *= kTeleDriveMaxAccelerationUnitsPerSecond;

        SmartDashboard.putNumber("xspeeddrivecommmand", xSpeed);
        SmartDashboard.putNumber("yspeeddrivecommmand", ySpeed);

        ChassisSpeeds chassisSpeeds;
        // Check if in field oriented mode
        if (!fieldOrientedFunction.get()) {
            SmartDashboard.putString("Mode", "Field Oriented");
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveDrive.getRotation2d());
        } else {
            SmartDashboard.putString("Mode", "Robot Oriented");
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        SmartDashboard.putNumber("xspeedchassis", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("yspeedchassis", chassisSpeeds.vyMetersPerSecond);

        // Calculate swerve module states
        SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveDrive.setModuleStates(moduleStates);

        SmartDashboard.putNumber("Swerve 1 angle", moduleStates[0].angle.getDegrees());
        SmartDashboard.putNumber("Swerve 2 angle", moduleStates[1].angle.getDegrees());
        SmartDashboard.putNumber("Swerve 3 angle", moduleStates[2].angle.getDegrees());
        SmartDashboard.putNumber("Swerve 4 angle", moduleStates[3].angle.getDegrees());
    }
}
