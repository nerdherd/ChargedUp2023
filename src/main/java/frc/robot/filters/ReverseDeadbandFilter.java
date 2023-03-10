package frc.robot.filters;

/**
 * A deadband filter that acts on the upper limit of input.
 * 
 * Also acts as a clamp.
 */
public class ReverseDeadbandFilter implements Filter {
    private double deadband;
    private double max;
    private double min;

    /**
     * Construct a new reverse deadband filter.
     * @param deadband  The deadband value
     * @param max       The maximum input
     * @param min       The minimum input
     */
    public ReverseDeadbandFilter(double deadband, double max, double min) {
        this.deadband = deadband;
        this.max = max;
        this.min = min;
    }

    public ReverseDeadbandFilter(double deadband) {
        this(deadband, 1, -1);
    }

    public double getDeadband() {
        return deadband;
    }

    public double calculate(double input) {
        if (input >= max - deadband) {
            return max;
        } else if (input <= min + deadband) {
            return min;
        } else {
            return input;
        }
    }

    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    public void setMax(double max) {
        this.max = max;
    }

    public void setMin(double min) {
        this.min = min;
    }

    
    
}
