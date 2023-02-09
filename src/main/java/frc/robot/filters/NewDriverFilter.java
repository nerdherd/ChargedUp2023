package frc.robot.filters;

import frc.robot.Constants.SwerveDriveConstants;

/**
 * New driver filter for swerve drive teleop input. Accepts a value in [-1, 1]
 */
public class NewDriverFilter extends FilterSeries {
    public NewDriverFilter(double deadband) {
        super(
            new DeadbandFilter(deadband),
            new WrapperFilter(
                (x) -> {
                    if (x  == 0) return 0.0;
                    if (x > 0) {
                        return x - deadband;
                    } else {
                        return x + deadband;
                    }
                }
            ),
            new PowerFilter(3),
            new WrapperFilter(
                (x) -> {
                    if (x == 0) return 0.0;
                    else if (x < 0) {
                        return x / ((1-deadband)*(1-deadband)) - deadband;
                    } else {
                        return x / ((1-deadband)*(1-deadband)) + deadband;
                    }
                }
            )
            // new ScaleFilter(SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond),
            // new ClampFilter(SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond)
        );
    }
}
