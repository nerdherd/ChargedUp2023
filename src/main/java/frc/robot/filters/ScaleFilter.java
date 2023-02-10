package frc.robot.filters;

/**
 * Multiplies an input by a constant scale.
 */
public class ScaleFilter implements Filter {
    private double scale;

    /**
     * Initialize a new Scale Filter.
     * @param scale
     */
    public ScaleFilter(double scale) {
        this.scale = scale;
    }

    public double calculate(double input) {
        return input * scale;
    }

    public void setScale(double scale) {
        this.scale = scale;
    }
}
