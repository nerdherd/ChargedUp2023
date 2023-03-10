package frc.robot.filters;

import java.util.function.Supplier;

/**
 * Normalizes a 2 by 2 square input to a circular input with radius 1.
 * 
 * For use with flight joysticks so that the magnitude of the input never exceeds 1
 */
public class NormalizationFilter implements Filter {
    static final double fourtyFive = Math.PI / 4;

    Supplier<Double> xInput;
    Supplier<Double> yInput;

    /**
     * Create a new normalization filter.
     * 
     * @param xInput    The x-value of the vector
     * @param yInput    The y-value of the vector
     */
    public NormalizationFilter(Supplier<Double> xInput, Supplier<Double> yInput) {
        this.xInput = xInput;
        this.yInput = yInput;
    }
    
    public double calculate(double input) {
        // Mapped to the first quadrant since magnitude is preserved
        // when reflected across the x and y axes
        double originalAngle = Math.atan2(Math.abs(yInput.get()), Math.abs(xInput.get()));

        // The angle mapped to [0, 45] 
        // [45, 90] gets mapped onto [45, 0] to preserve the magnitude
        double translatedAngle = fourtyFive - Math.abs(originalAngle - fourtyFive);
        
        // 1 / sec(angle) is the magnitude of the vector that extends to the square
        // Dividing by the magnitude scales the input down to a circular input
        double scale = Math.cos(translatedAngle);
        return input * scale;
    }

    public void setXInput(Supplier<Double> xInput) {
        this.xInput = xInput;
    }

    public void setYInput(Supplier<Double> yInput) {
        this.yInput = yInput;
    }
}
