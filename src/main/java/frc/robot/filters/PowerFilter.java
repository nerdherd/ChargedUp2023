package frc.robot.filters;

public class PowerFilter implements Filter {
    private int power;

    public PowerFilter(int power) {
        this.power = power;
    }

    public double calculate(double input) {
        return Math.signum(input) * Math.abs(Math.pow(input, power));
    }
}
