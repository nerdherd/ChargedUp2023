package frc.robot.filters;

public class ScaleFilter implements Filter {
    private double scale;
    public ScaleFilter(double scale) {
        this.scale = scale;
    }

    public double calculate(double input) {
        return input * scale;
    }
}
