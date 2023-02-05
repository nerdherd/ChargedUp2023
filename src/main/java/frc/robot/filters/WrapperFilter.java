package frc.robot.filters;

import java.util.function.Function;

public class WrapperFilter implements Filter {
    private Function<Double, Double> filterFunction;
    
    /**
     * Create a wrapper filter for a function.
     * @param filterFunction    The function to wrap as a filter
     */
    public WrapperFilter(Function<Double, Double> filterFunction) {
        this.filterFunction = filterFunction;
    }

    public double calculate(double input) {
        return filterFunction.apply(input);
    }
}
