package frc.robot.filters;

public class DeadbandFilter implements Filter {
    private double deadband;

    public double getDeadband() {
        return deadband;
    }

    public double calculate(double input) {
        if (Math.abs(input) >= Math.abs(deadband)) {
            return input;
        } else {
            return 0;
        }
    }

    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    public DeadbandFilter(double deadband) {
        this.deadband = deadband;
    }
    
}
