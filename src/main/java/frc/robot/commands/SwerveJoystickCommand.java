package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.filters.Filter;
import frc.robot.filters.NewDriverFilter;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;

import static frc.robot.Constants.SwerveDriveConstants.*;

public class SwerveJoystickCommand extends CommandBase {
    private final SwerveDrivetrain swerveDrive;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final Supplier<Boolean> towSupplier, precisionSupplier;
    private final Supplier<Boolean> dodgeSupplier;
    private final Supplier<Double> desiredAngle;
    private final Supplier<Boolean> turnToAngleSupplier;
    private final PIDController turnToAngleController;
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
     * @param dodgeDirectionSupplier A supplier that supplies the dodge direction.
     * @param precisionSupplier     A boolean supplier that toggles the precision mode.
     */
    public SwerveJoystickCommand(SwerveDrivetrain swerveDrive,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, 
            Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> towSupplier, 
            Supplier<Boolean> dodgeSupplier, 
            Supplier<DodgeDirection> dodgeDirectionSupplier, 
            Supplier<Boolean> precisionSupplier,
            Supplier<Boolean> turnToAngleSupplier, 
            Supplier<Double> desiredAngleSupplier
        ) {
        this.swerveDrive = swerveDrive;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.towSupplier = towSupplier;

        this.dodgeSupplier = dodgeSupplier;
        this.dodgeDirectionSupplier = dodgeDirectionSupplier;

        this.precisionSupplier = precisionSupplier;

        this.turnToAngleSupplier = turnToAngleSupplier;
        this.desiredAngle = desiredAngleSupplier;

        this.xFilter = new NewDriverFilter(
            ControllerConstants.kDeadband, 
            kMinimumMotorOutput,
            kTeleDriveMaxSpeedMetersPerSecond, 
            kDriveAlpha, 
            kTeleMaxAcceleration, 
            kTeleMaxDeceleration);
        this.yFilter = new NewDriverFilter(
            ControllerConstants.kDeadband, 
            kMinimumMotorOutput,
            kTeleDriveMaxSpeedMetersPerSecond, 
            kDriveAlpha, 
            kTeleMaxAcceleration, 
            kTeleMaxDeceleration);
        this.turningFilter = new NewDriverFilter(
            ControllerConstants.kRotationDeadband, 
            kMinimumMotorOutput,
            kTeleDriveMaxAngularSpeedRadiansPerSecond, 
            kDriveAlpha, 
            kTeleMaxAcceleration, 
            kTeleMaxDeceleration);
        
        this.turnToAngleController = new PIDController(
            SwerveAutoConstants.kPTurnToAngle, 
            SwerveAutoConstants.kITurnToAngle, 
            SwerveAutoConstants.kDTurnToAngle, 
            0.02);
        
        this.turnToAngleController.setTolerance(
            SwerveAutoConstants.kTurnToAnglePositionToleranceAngle, 
            SwerveAutoConstants.kTurnToAngleVelocityToleranceAnglesPerSec * 0.02);
        
        this.turnToAngleController.enableContinuousInput(0, 360);

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
        double turningSpeed;
        double xSpeed = xSpdFunction.get();
        double ySpeed = -ySpdFunction.get();

        double filteredTurningSpeed;
        double filteredXSpeed = xFilter.calculate(xSpeed);
        double filteredYSpeed = yFilter.calculate(ySpeed);

        // Turn to angle
        if (turnToAngleSupplier.get()) {
            double targetAngle = desiredAngle.get();
            turningSpeed = turnToAngleController.calculate(swerveDrive.getImu().getHeading(), targetAngle);
            turningSpeed = Math.toRadians(turningSpeed);
            turningSpeed = MathUtil.clamp(
                turningSpeed, 
                -SwerveDriveConstants.kTurnToAngleMaxAngularSpeedRadiansPerSecond, 
                SwerveDriveConstants.kTurnToAngleMaxAngularSpeedRadiansPerSecond);
            filteredTurningSpeed = turningSpeed;
            xSpeed += 0.01;
            ySpeed += 0.01;
        } else {
            // Manual turning
            turningSpeed = turningSpdFunction.get();
            turningSpeed *= -1;
            filteredTurningSpeed = turningFilter.calculate(turningSpeed);
        }

        if (precisionSupplier.get()) {
            filteredXSpeed /= 4;
            filteredYSpeed /= 4;
            filteredTurningSpeed /= 4; // Also slows down the turn to angle speed
        }
        
        ChassisSpeeds chassisSpeeds;
        // Check if in field oriented mode
        if (!fieldOrientedFunction.get()) {
            swerveDrive.setDriveMode(DRIVE_MODE.FIELD_ORIENTED);
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                filteredXSpeed, filteredYSpeed, filteredTurningSpeed, 
                swerveDrive.getImu().getRotation2d());
        } else {
            swerveDrive.setDriveMode(DRIVE_MODE.ROBOT_ORIENTED);
            chassisSpeeds = new ChassisSpeeds(
                filteredXSpeed, filteredYSpeed, filteredTurningSpeed);
        }
                
        // SmartDashboard.putBoolean("Turn to angle", turnToAngleSupplier.get());
        // SmartDashboard.putNumber("Swerve Drive Target Angle", desiredAngle.get());
        // SmartDashboard.putNumber("Turning speed", filteredTurningSpeed);

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
                filteredTurningSpeed = 3;
                rotationCenter = kRotationCenters[kRightRotationCenters[quadrant]];
            } else {
                filteredTurningSpeed = -3;
                rotationCenter = kRotationCenters[kLeftRotationCenters[quadrant]];
            }

            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                0, 0, filteredTurningSpeed, 
                swerveDrive.getImu().getRotation2d());

            SmartDashboard.putNumber("Rotation Center X", rotationCenter.getX());
            SmartDashboard.putNumber("Rotation Center Y", rotationCenter.getY());
            SmartDashboard.putNumber("Dodge Quadrant", quadrant);
            moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds, rotationCenter);
        } else {
            robotOrientedJoystickDirection = null;
            moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        }

        
        // Calculate swerve module states
        swerveDrive.setModuleStates(moduleStates);
    }
}
