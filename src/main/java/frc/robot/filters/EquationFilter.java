package frc.robot.filters;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import net.objecthunter.exp4j.Expression;
import net.objecthunter.exp4j.ExpressionBuilder;

public class EquationFilter implements Filter {
    private Expression expression;
    private String equation;
    private boolean isValid;

    /**
     * Construct an equation filter from array values.
     * <p>
     * Use for creating equation filters from networktables values.
     * 
     * @param equation
     * @param variables
     * @param values
     * @return
     */
    public EquationFilter(String equation, String[] variables, double[] values) {
        setEquation(equation, variables, values);
    }

    /**
     * Construct a new EquationFilter with no variables.
     * @param equation  The equation string.
     */
    public EquationFilter(String equation) {
        setEquation(equation, new HashMap<String, Double>());
    }

    /**
     * Construct a new EquationFilter with variables.
     * @param equation  The equation string.
     * @param variables A map of the variable names to values.
     */
    public EquationFilter(String equation, HashMap<String, Double> variables) {
        setEquation(equation, variables);
    }

    /**
     * Set the equation with variables.
     * @param equation
     * @param variables
     */
    public void setEquation(String equation, HashMap<String, Double> variables) {
        this.equation = equation;

        ExpressionBuilder builder = new ExpressionBuilder(equation);
        if (variables != null) {
            builder.variables(variables.keySet());
        }

        if (variables.containsKey("x")) {
            DriverStation.reportWarning("Variable x provided to equation filter will be overriden by input.", null);
        } else {
            builder.variable("x");
        }

        this.expression = builder.build();
        this.isValid = expression.validate().isValid();

        if (!this.isValid) {
            DriverStation.reportWarning("Invalid expression provided to equation filter.", true);
        }
    }

    /**
     * Set the equation without variables.
     * @param equation
     * @param variables
     */
    public void setEquation(String equation) {
        setEquation(equation, null);
    }

    /**
     * Set the equation from two arrays of variables.
     * @param equation
     * @param variables
     * @param values
     */
    public void setEquation(String equation, String[] variables, double[] values) {
        if (variables == null || variables.length == 0 || values == null || values.length == 0) {
            setEquation(equation);
            return;
        }

        setEquation(equation, new HashMap<String, Double>() {{   
            for (int i = 0; i<variables.length; i++) {
                put(variables[i], values[i]);
            }
        }});
    }

    public double calculate(double input) {
        expression.setVariable("x", input);

        if (!this.isValid) {
            DriverStation.reportWarning("Invalid expression provided to equation filter.", true);
            return 0;
        }

        return expression.evaluate();
    }

    @Override
    public String toString() {
        return this.equation;
    }
}
