package frc.robot.util.filters;

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

    /**
     * Initialize an empty wrapper filter (returns the input as output)
     */
    public WrapperFilter() {
        this.filterFunction = (x) -> x;
    }

    /**
     * Set the filter function for the wrapper filter.
     */
    public void setFilterFunction(Function<Double, Double> filterFunction) {
        this.filterFunction = filterFunction;
    }

    public double calculate(double input) {
        return filterFunction.apply(input);
    }
}
