package frc.robot.filters;

import frc.robot.util.NerdyMath;

/** Clamps an input between two values. */
public class ClampFilter implements Filter {
    private double max;
    private double min;

    public ClampFilter(double max, double min) {
        this.max = max;
        this.min = min;
    }

    public double calculate(double input) {
        return NerdyMath.clamp(input, min, max);
    }
    

    public double getMax() {
        return max;
    }
    public void setMax(double max) {
        this.max = max;
    }

    public double getMin() {
        return min;
    }
    public void setMin(double min) {
        this.min = min;
    }

    
}