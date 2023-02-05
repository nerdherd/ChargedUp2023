package frc.robot.filters;

public class FilterSeries implements Filter {
    private Filter[] filters;

    public FilterSeries(Filter... filters) {
        this.filters = filters;
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

    public void setFilters(Filter[] filters) {
        this.filters = filters;
    }
}
