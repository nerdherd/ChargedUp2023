package frc.robot.filters;

import frc.robot.Constants.SwerveDriveConstants;

public class LowPassFilter implements Filter {
    private double alpha;
    private double oneMinusAlpha;
    private double lastInput;

    public LowPassFilter(double alpha, double oneMinusAlpha, double start) {
        this.alpha = alpha;
        this.oneMinusAlpha = oneMinusAlpha;
        this.lastInput = start;
    }

    public LowPassFilter(double alpha, double oneMinusAlpha) {
        this(alpha, oneMinusAlpha, 0);
    }

    public LowPassFilter(double alpha) {
        this(alpha, 1-alpha, 0);
    }

    public LowPassFilter() {
        this(SwerveDriveConstants.kDriveAlpha, SwerveDriveConstants.kDriveOneMinusAlpha, 0);
    }

    public double calculate(double input) {
        double output = (alpha * input) + (oneMinusAlpha * lastInput);
        lastInput = input;
        return output;
    }
}
