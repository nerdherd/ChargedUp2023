package frc.robot.filters;

/**
 * A series of filters that calculate in order
 */
public class FilterSeries implements Filter {
    private Filter[] filters;

    public FilterSeries(Filter... filters) {
        this.filters = filters;
    }

    /**
     * Create a filter series with an empty filter.
     */
    public FilterSeries() {
        this.filters = new Filter[] {new WrapperFilter()};
    }

    /**
     * Calculate the input filtered by all filters in the series.
     * @param input     The input to filter
     * @return output   The output of the filters
     */
    public double calculate(double input) {
        double output = input;
        for (Filter filter : filters) {
            output = filter.calculate(output);
        }

        return output;
    }

    public Filter[] getFilters() {
        return filters;
    }

    public void setFilters(Filter... filters) {
        this.filters = filters;
    }
}
