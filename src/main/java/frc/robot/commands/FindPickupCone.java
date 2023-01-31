package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;

public class FindPickupCone extends CommandBase{
    boolean isOnGround = true;
    private PIDController XSpeedController;
    private PIDController YSpeedController;
    private PIDController TurnSpeedController;

    public FindPickupCone(SwerveDrivetrain swerveDrive, Limelight limelight, boolean isOnGround) {
        limelight.setPipeline(1);
        this.isOnGround = isOnGround; // Use later
        double buffer = 0.05;

        XSpeedController = new PIDController(0, 0, 0);
        YSpeedController = new PIDController(0, 0, 0);
        TurnSpeedController = new PIDController(0, 0, 0);

        if(limelight.hasValidTarget()) {
            if(limelight.getArea() > 0.5 - buffer) {

            }
            else if(limelight.getArea() < 0.5 + buffer) {

            }
            else {
                
            }
        }
        swerveDrive.drive(0, 0, 0);
    }

    public FindPickupCone(SwerveDrivetrain swerveDrive, Limelight limelight) {
        
    }
}
