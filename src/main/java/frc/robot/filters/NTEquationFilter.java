package frc.robot.filters;

import java.util.Arrays;
import java.util.function.Supplier;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * An equation filter that updates based on NetworkTables values.
 * <p>
 * Use {@link #startUpdateLoop()} to have it update automatically.
 */
public class NTEquationFilter extends EquationFilter {
    private StringSubscriber equation;
    private StringArraySubscriber variables;
    private DoubleArraySubscriber values;

    private String lastEquation;
    private String[] lastVariables;
    private double[] lastValues;

    private Trigger updateTrigger;

    private double lastUpdateTimestamp;

    public NTEquationFilter(StringSubscriber equation, 
            StringArraySubscriber variables, 
            DoubleArraySubscriber values, 
            Trigger updateTrigger) {
        super(
            equation.get("x"), 
            variables.get(new String[0]), 
            values.get(new double[0]));
        
        this.equation = equation;
        this.variables = variables;
        this.values = values;
        
        lastEquation = equation.get();
        lastVariables = variables.get();
        lastValues = values.get();

        this.updateTrigger = updateTrigger;

        this.lastUpdateTimestamp = WPIUtilJNI.now() * 1e-6;
    }

    public NTEquationFilter(StringSubscriber equation, 
            StringArraySubscriber variables, 
            DoubleArraySubscriber values,
            Supplier<Boolean> updater) {
        this(equation, variables, values, new Trigger(updater::get));
    }

    public NTEquationFilter(StringSubscriber equation, 
            StringArraySubscriber variables, 
            DoubleArraySubscriber values, 
            BooleanSubscriber updater) {
        this(equation, variables, values, new Trigger(() -> updater.get(false)));
    }

    /**
     * Start the update loop.
     */
    public void startUpdateLoop() {
        this.updateTrigger.onTrue(Commands.runOnce(this::checkedUpdate));
    }

    /**
     * Stop the update loop.
     */
    public void stopUpdateLoop() {
        this.updateTrigger.onTrue(Commands.runOnce(() -> {}));
    }

    /**
     * Update the equation if the values have changed.
     */
    public void checkedUpdate() {
        if (!checkValuesChanged()) return;
        update();
    }

    /**
     * Update the equation (regardless of value changes)
     */
    public void update() {
        this.lastEquation = this.equation.get();
        this.lastVariables = this.variables.get();
        this.lastValues = this.values.get();
        setEquation(lastEquation, lastVariables, lastValues);

        this.lastUpdateTimestamp = WPIUtilJNI.now() * 1e-6;
    }


    /**
     * Check if the values have changed since the last update.
     * @return
     */
    private boolean checkValuesChanged() {
        if (lastEquation != equation.get()) return true;
        if (!Arrays.equals(lastVariables, variables.get())) return true;
        if (!Arrays.equals(lastValues, values.get())) return true;
        return false;
    }

    /**
     * Get the timestamp of the last update.
     * 
     * @return  The timestamp
     */
    public double getLastUpdate() {
        return this.lastUpdateTimestamp;
    }
}
