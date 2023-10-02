package frc.robot.util.filters;

/**
 * An exponential smoothing filter 
 * (see {@link https://en.wikipedia.org/wiki/Exponential_smoothing})
 * <p>
 * Uses the equation: 
 * <math display="block" style="display:inline-block;">
 * <mrow>
 *      <mi>α</mi>
 *      <msub>
 *          <mi>x</mi>
 *          <mi>t</mi>
 *      </msub>
 *      <mo>+</mo>
 *      <mo form="prefix" stretchy="false">(</mo>
 *      <mn>1</mn>
 *      <mo>−</mo>
 *      <mi>α</mi>
 *      <mo form="postfix" stretchy="false">)</mo>
 *      <msub>
 *          <mi>x</mi>
 *          <mrow>
 *              <mi>t</mi>
 *              <mo>−</mo>
 *              <mn>1</mn>
 *          </mrow>
 *      </msub>
 *  </mrow>
 *  </math>
 * <p>
 * Taken from 687's Rapid React 2022 drive command, 
 * where it is labeled as low pass filter
 * {@link https://github.com/nerdherd/RapidReact2022}.
 */
public class ExponentialSmoothingFilter implements Filter {
    private double alpha;
    private double oneMinusAlpha;
    private double lastInput;

    public ExponentialSmoothingFilter(double alpha, double oneMinusAlpha, double start) {
        this.alpha = alpha;
        this.oneMinusAlpha = oneMinusAlpha;
        this.lastInput = start;
    }

    public ExponentialSmoothingFilter(double alpha, double oneMinusAlpha) {
        this(alpha, oneMinusAlpha, 0);
    }

    public ExponentialSmoothingFilter(double alpha) {
        this(alpha, 1-alpha, 0);
    }

    public double calculate(double input) {
        double output = (alpha * input) + (oneMinusAlpha * lastInput);
        lastInput = output;
        return output;
    }
}
