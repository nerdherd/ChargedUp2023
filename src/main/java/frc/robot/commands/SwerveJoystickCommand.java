package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.SwerveDriveConstants.*;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrivetrain;

public class SwerveJoystickCommand extends CommandBase {
    private final SwerveDrivetrain swerveDrive;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final Supplier<Boolean> towSupplier, dodgeSupplier;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private Translation2d rotationCenter;
    private PIDController pidController;
    private double prevXInput, prevYInput, prevTurningSpeed = 0.0;
    private double targetAngle = 180;

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
            Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> towSupplier, Supplier<Boolean> dodgeSupplier) {
        this.swerveDrive = swerveDrive;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.towSupplier = towSupplier;
        this.dodgeSupplier = dodgeSupplier;
        this.xLimiter = new SlewRateLimiter(kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveDrive);

        this.pidController = new PIDController(
            SwerveAutoConstants.kPTurnToAngle, 
            SwerveAutoConstants.kITurnToAngle, 
            SwerveAutoConstants.kDTurnToAngle, 
            0.02);
        
        this.pidController.setTolerance(
            SwerveAutoConstants.kTurnToAnglePositionToleranceAngle, 
            SwerveAutoConstants.kTurnToAngleVelocityToleranceAnglesPerSec * 0.02);
        
        this.pidController.enableContinuousInput(0, 360);
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

    public void setTargetAngle(double angle) {
        this.targetAngle = angle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    @Override
    public void execute() {
        if (towSupplier.get()) {
            swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates);
            return;
        }

        double turningSpeed = turningSpdFunction.get();

        // if (this.turnToAngleSupplier.get()) {
        //     // Calculate turning speed with PID
        //     turningSpeed = pidController.calculate(swerveDrive.getHeading(), targetAngle);
        //     turningSpeed = Math.toRadians(turningSpeed);
        //     turningSpeed = NerdyMath.clamp(
        //         turningSpeed, 
        //         -SwerveDriveConstants.kTurnToAngleMaxAngularSpeedRadiansPerSecond, 
        //         SwerveDriveConstants.kTurnToAngleMaxAngularSpeedRadiansPerSecond);
        // } else {
        //     turningSpeed = turningSpdFunction.get();
        // }

        // get speeds
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();

        double originalXSpeed = xSpeed;
        double originalYSpeed = ySpeed;

        // Apply deadband to the speeds
        xSpeed = applyDeadband(xSpeed);
        ySpeed = applyDeadband(ySpeed);
        turningSpeed = applyDeadband(turningSpeed);
        
        // Apply low pass filter
        xSpeed = (kDriveAlpha * xSpeed) + (kDriveOneMinusAlpha * prevXInput);
        ySpeed = (kDriveAlpha * ySpeed) + (kDriveOneMinusAlpha * prevYInput);
        turningSpeed = (kDriveAlpha * turningSpeed)
        + (kDriveOneMinusAlpha * prevTurningSpeed);
        
        prevXInput = xSpeed;
        prevYInput = ySpeed;
        prevTurningSpeed = turningSpeed;
        
        // Apply quadratic
        // xSpeed = Math.signum(xSpeed) * xSpeed * xSpeed;
        // ySpeed = Math.signum(ySpeed) * ySpeed * ySpeed;
        
        // Apply cubic
        xSpeed = Math.signum(xSpeed) * Math.abs(xSpeed * xSpeed * xSpeed);
        ySpeed = Math.signum(ySpeed) * Math.abs(ySpeed * ySpeed * ySpeed);
        turningSpeed = Math.signum(turningSpeed) * turningSpeed * turningSpeed;
        
        // Apply the slew rate limiter to the speeds
        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        turningSpeed = turningLimiter.calculate(turningSpeed);
        
        // Convert speeds to meters per second
        xSpeed *= kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed *= kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed *= kTeleDriveMaxAngularSpeedRadiansPerSecond;
        
        SmartDashboard.putNumber("xspeeddrivecommmand", xSpeed);
        SmartDashboard.putNumber("yspeeddrivecommmand", ySpeed);
        
        
        ChassisSpeeds chassisSpeeds;
        // Check if in field oriented mode
        if (!fieldOrientedFunction.get()) {
            SmartDashboard.putString("Mode", "Field Oriented");
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveDrive.getImu().getRotation2d());
        } else {
            SmartDashboard.putString("Mode", "Robot Oriented");
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        SmartDashboard.putNumber("xspeedchassis", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("yspeedchassis", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Turning speed", turningSpeed);

        SwerveModuleState[] moduleStates;

        if (dodgeSupplier.get()) {
            if (rotationCenter == null) {
                rotationCenter = new Translation2d(kRotationOffset, 
                    // Rotation 2d is measured counterclockwise from the right vector
                    // Y speed = left/right = x component
                    // X speed = forward/back = y component
                    new Rotation2d(originalYSpeed, originalXSpeed)
                        .rotateBy(Rotation2d.fromDegrees(
                            // imu is measured clockwise from forward vector
                            swerveDrive.getImu().getHeading()))
                        );
            }
            // Might need to swap x and y on rotation center depending on how it gets interpreted
            // rotationCenter = new Translation2d(rotationCenter.getX(), rotationCenter.getY());
            moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds, rotationCenter);
        } else {
            rotationCenter = null;
            moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        }

        // Calculate swerve module states
        swerveDrive.setModuleStates(moduleStates);
    }
}
