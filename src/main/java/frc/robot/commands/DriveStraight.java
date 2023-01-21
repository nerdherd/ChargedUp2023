// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.NerdyMath;

public class DriveStraight extends CommandBase {
  private Drivetrain drive;
  private double maxForwardDriveSpeed;
  private double distanceMeter;
  private double heading;
  private boolean continueMove;
  private boolean finished;

  /** Creates a new DriveStraight. */
  public DriveStraight(Drivetrain drive, 
                        double maxForwardDriveSpeed,
                        double distanceMeter,
                        double heading,
                        boolean continueMove
                        ) {
    this.drive = drive;
    this.maxForwardDriveSpeed = maxForwardDriveSpeed;
    this.distanceMeter = distanceMeter;
    this.heading = heading;
    this.continueMove = continueMove;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean hasInitStraight = false;
    //PIDController forwardControllerImu = new PIDController(0.1, 0, 0);
    PIDController turnControllerImu = new PIDController(0.001, 0, 0);

    double ticks2Go = 0; // how many encode ticks to move
    double ticks2SlowDown = 0; // when to slow so you don't overshoot

    double drivePower = 0;
        if(!hasInitStraight) {
            hasInitStraight = true;
            drivePower = 0.3;
            ticks2Go = drive.meterToTicks(distanceMeter);//inches2Ticks(distance); // set up encoder stop condition
            ticks2SlowDown = ticks2Go*0.8;//inches2Ticks(distance*0.2); // set up encoder slow down condition
        }

        double position = drive.getTicks();
        if (position >= ticks2SlowDown)
            drivePower = 0.15; // cut power prepare to stop

        if (position >= ticks2Go) { // reached desired encoder position
            if (continueMove) {
                drive.setPower(0.1,0.1);
            } 
            else {
                drive.setPower(0, 0);
            }
            finished = true;
        } 
        else { // move straight
            double rotateToAngleRate = turnControllerImu.calculate(drive.getHeading(), heading); // calc error correction
            //tankDriveLeftSpeed = (drivePower + rotateToAngleRate);
            //tankDriveRightSpeed = (drivePower - rotateToAngleRate);
            double tankDriveLeftSpeed = NerdyMath.clamp((drivePower + rotateToAngleRate), -maxForwardDriveSpeed, maxForwardDriveSpeed);
            double tankDriveRightSpeed = NerdyMath.clamp((drivePower - rotateToAngleRate), -maxForwardDriveSpeed, maxForwardDriveSpeed);
            drive.tankDrive(tankDriveLeftSpeed*1.5, tankDriveRightSpeed*1.5);
            
            finished = false;
        } 
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
