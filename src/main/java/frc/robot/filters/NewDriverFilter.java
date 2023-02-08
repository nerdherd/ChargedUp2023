package frc.robot.filters;

/**
 * New driver filter for swerve drive teleop input. Accepts a value in [-1, 1]
 */
public class NewDriverFilter extends FilterSeries {
    public NewDriverFilter() {
        super(
            new DeadbandFilter(0.05),
            new WrapperFilter(
                (x) -> x - 0.05
            ),
            new PowerFilter(3),
            new WrapperFilter(
                (x) -> 0.05 + x / (0.95*0.95)
            )
        );
    }
}
