package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.SwerveDriveConstants.*;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.filters.Filter;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.filters.NewDriverFilter;

public class SwerveJoystickCommand extends CommandBase {
    private final SwerveDrivetrain swerveDrive;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final Supplier<Boolean> towSupplier, dodgeSupplier, precisionSupplier;
    private Filter xFilter, yFilter, turningFilter;
    private Translation2d robotOrientedJoystickDirection;
    private Supplier<DodgeDirection> dodgeDirectionSupplier;

    public enum DodgeDirection {
        LEFT,
        RIGHT,
        NONE
    }

    /**
     * Construct a new SwerveJoystickCommand
     * 
     * @param swerveDrive           The Swerve Drive subsystem
     * @param xSpdFunction          A supplier returning the desired x speed
     * @param ySpdFunction          A supplier returning the desired y speed
     * @param turningSpdFunction    A supplier returning the desired turning speed
     * @param fieldOrientedFunction A boolean supplier that toggles field oriented/robot oriented mode.
     * @param towSupplier           A boolean supplier that toggles the tow mode.
     * @param dodgeSupplier         A boolean supplier that toggles the dodge mode.
     */
    public SwerveJoystickCommand(SwerveDrivetrain swerveDrive,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> towSupplier, Supplier<Boolean> dodgeSupplier, 
            Supplier<DodgeDirection> dodgeDirectionSupplier, Supplier<Boolean> precisionSupplier) {
        this.swerveDrive = swerveDrive;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.towSupplier = towSupplier;
        this.dodgeDirectionSupplier = dodgeDirectionSupplier;
        this.precisionSupplier = precisionSupplier;

        // Old filters
        
        // this.xFilter = new DriverFilter(
        //     OIConstants.kDeadband, 
        //     kDriveAlpha,
        //     kDriveOneMinusAlpha, 
        //     kTeleDriveMaxSpeedMetersPerSecond,
        //     kTeleMaxAcceleration,
        //     3, kTeleMaxDeceleration);
        // this.yFilter = new DriverFilter(
        //     OIConstants.kDeadband, 
        //     kDriveAlpha,
        //     kDriveOneMinusAlpha, 
        //     kTeleDriveMaxSpeedMetersPerSecond,
        //     kTeleMaxAcceleration,
        //     3, kTeleMaxDeceleration);
        // this.turningFilter = new DriverFilter(
        //     OIConstants.kDeadband, 
        //     kDriveAlpha,
        //     kDriveOneMinusAlpha, 
        //     kTeleDriveMaxSpeedMetersPerSecond,
        //     kTeleMaxAcceleration,
        //     3, kTeleMaxDeceleration);

        // New filters

        this.xFilter = new NewDriverFilter(
            OIConstants.kDeadband, 
            kMinimumMotorOutput,
            kTeleDriveMaxSpeedMetersPerSecond, 
            kDriveAlpha, 
            kTeleMaxAcceleration, 
            kTeleMaxDeceleration);
        this.yFilter = new NewDriverFilter(
            OIConstants.kDeadband, 
            kMinimumMotorOutput,
            kTeleDriveMaxSpeedMetersPerSecond, 
            kDriveAlpha, 
            kTeleMaxAcceleration, 
            kTeleMaxDeceleration);
        this.turningFilter = new NewDriverFilter(
            OIConstants.kRotationDeadband, 
            kMinimumMotorOutput,
            kTeleDriveMaxAngularSpeedRadiansPerSecond, 
            kDriveAlpha, 
            kTeleMaxAcceleration, 
            kTeleMaxDeceleration);

        // this.xFilter = new FilterSeries(
        //     new DeadbandFilter(OIConstants.kDeadband),
        //     new ScaleFilter(kTeleDriveMaxSpeedMetersPerSecond)
        // );
        
        // this.yFilter = new FilterSeries(
        //     new DeadbandFilter(OIConstants.kDeadband),
        //     new ScaleFilter(kTeleDriveMaxSpeedMetersPerSecond)
        // );
        // this.turningFilter = new FilterSeries(
        //     new DeadbandFilter(OIConstants.kDeadband),
        //     new ScaleFilter(kTeleDriveMaxAngularSpeedRadiansPerSecond)
        // );

        this.dodgeSupplier = dodgeSupplier;

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (towSupplier.get()) {
            swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates);
            return;
        }

        // get speeds
        double turningSpeed = turningSpdFunction.get();
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();

        double filteredTurningSpeed = turningFilter.calculate(turningSpeed);
        double filteredXSpeed = xFilter.calculate(xSpeed);
        double filteredYSpeed = yFilter.calculate(ySpeed);
        
        if (precisionSupplier.get()) {
            filteredTurningSpeed /= 2;
            filteredXSpeed /= 2;
            filteredYSpeed /= 2;
        }
        
        ChassisSpeeds chassisSpeeds;
        // Check if in field oriented mode
        if (!fieldOrientedFunction.get()) {
            SmartDashboard.putString("Mode", "Field Oriented");
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                filteredXSpeed, filteredYSpeed, filteredTurningSpeed, 
                swerveDrive.getImu().getRotation2d());
            } else {
            SmartDashboard.putString("Mode", "Robot Oriented");
            chassisSpeeds = new ChassisSpeeds(
                filteredXSpeed, filteredYSpeed, filteredTurningSpeed);
        }
                
        SmartDashboard.putNumber("Swerve Drive X Speed", filteredXSpeed);
        SmartDashboard.putNumber("Swerve Drive Y Speed", filteredYSpeed);
        SmartDashboard.putNumber("Swerve Drive X Chassis", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Swerve Drive Y Chassis", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Turning speed", filteredTurningSpeed);

        SwerveModuleState[] moduleStates;

        if (dodgeSupplier.get()) {
            if (robotOrientedJoystickDirection == null) {
                robotOrientedJoystickDirection = new Translation2d(kRotationOffset, 
                    // Rotation 2d is measured counterclockwise from the right vector
                    // Y speed = left/right = x component
                    // X speed = forward/back = y component
                    new Rotation2d(ySpeed, xSpeed)
                        .rotateBy(Rotation2d.fromDegrees(
                            // imu is measured clockwise from forward vector
                            swerveDrive.getImu().getHeading()))
                        );
                // Might need to swap x and y on rotation center depending on how it gets interpreted
                // robotOrientedJoystickDirection = new Translation2d(robotOrientedJoystickDirection.getY(), robotOrientedJoystickDirection.getX());
                SmartDashboard.putNumber("Dodge X", robotOrientedJoystickDirection.getX());
                SmartDashboard.putNumber("Dodge Y", robotOrientedJoystickDirection.getY());
            }

            if (robotOrientedJoystickDirection.getX() > 0) {
                filteredTurningSpeed *= -1;
            }

            // Forward = quadrant 0, Right = quadrant 1, Down = quadrant 2, Left = quadrant 3
            double angle = MathUtil.inputModulus(robotOrientedJoystickDirection.getAngle().getDegrees(), -45, 315) + 45;
            int quadrant = (int) (angle / 90) % 4;

            Translation2d rotationCenter;

            DodgeDirection dodgeDirection = dodgeDirectionSupplier.get();
            if (dodgeDirection == DodgeDirection.LEFT) {
                rotationCenter = kRotationCenters[kLeftRotationCenters[quadrant]];
            } else {
                rotationCenter = kRotationCenters[kRightRotationCenters[quadrant]];
            }

            moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds, rotationCenter);
        } else {
            robotOrientedJoystickDirection = null;
            moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        }

        
        // Calculate swerve module states
        swerveDrive.setModuleStates(moduleStates);
    }
}
