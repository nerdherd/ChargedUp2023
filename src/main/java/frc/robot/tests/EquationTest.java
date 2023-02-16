package frc.robot.tests;

import java.util.HashMap;

import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.filters.EquationFilter;

public class EquationTest {
    HashMap<String, Double> variables = new HashMap<String, Double>() {{
        put("e", Math.E);   
    }};

    EquationFilter filter = new EquationFilter("e^(pi*x)", variables);

    public void initialize() {
        SmartDashboard.putNumber("Input", 0);
    }

    public void testPeriodic() {
        double startTime = WPIUtilJNI.now() * 1e6;

        double input = SmartDashboard.getNumber("Input", 0);
        double output = filter.calculate(input);
        SmartDashboard.putNumber("Output", output);

        double endTime = WPIUtilJNI.now() * 1e6;

        SmartDashboard.putNumber("Delay", endTime - startTime);
    }
}
