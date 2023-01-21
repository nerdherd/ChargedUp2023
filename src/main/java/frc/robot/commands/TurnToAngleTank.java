package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.NerdyMath;

public class TurnToAngleTank extends CommandBase{
    private double heading;
    private Drivetrain drive;
    private PIDController pidController;
    private boolean finished;

    public TurnToAngleTank(Drivetrain drive, double heading) {
        // Default period is 20 ms
        this.drive = drive;
        this.heading = heading;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Calculate turning speed with PID
        PIDController turnControllerProfiledImu =
        new PIDController(0.009, 0.0, 0);
        double[] turnLog = new double[3];
        double angleYawRobot;
        angleYawRobot = drive.getHeading()%360;
        double turningSpeed = turnControllerProfiledImu.calculate(drive.getHeading(), heading);
        turningSpeed = NerdyMath.clamp(turningSpeed, -0.5, 0.5);
        double tankDriveLeftSpeed = turningSpeed;
        double tankDriveRightSpeed = -turningSpeed;
        drive.setPower(tankDriveLeftSpeed, tankDriveRightSpeed);
        turnLog[0] = turnControllerProfiledImu.getPositionError();
        turnLog[1] = turningSpeed;
        turnLog[2] = drive.getHeading();
        if(NerdyMath.inRangeLess(drive.getHeading(), heading-1, heading+1)) {
            finished = true;
        }
        else{
            finished = false;
        }
    }
    

    @Override
    public boolean isFinished() {
        return finished;
    }
}