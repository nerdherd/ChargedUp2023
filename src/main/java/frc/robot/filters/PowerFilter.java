package frc.robot.filters;

/**
 * Raises an input to a power.
 */
public class PowerFilter implements Filter {
    private int power;

    public PowerFilter(int power) {
        this.power = power;
    }

    public double calculate(double input) {
        return Math.signum(input) * Math.abs(Math.pow(input, power));
    }
}
