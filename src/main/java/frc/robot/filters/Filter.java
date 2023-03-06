package frc.robot.filters;

public interface Filter {
    /**
     * Calculate the next output from the input.
     * @param input The input variable
     * @return The output of the filter
     */
    public double calculate(double input);
}
