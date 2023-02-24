// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.legacy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrivetrain;
import frc.robot.util.NerdyMath;

public class DriveStraight extends CommandBase {
  private TankDrivetrain drive;
  private double maxForwardDriveSpeed;
  private double distanceMeter;
  private double heading;
  private boolean continueMove;
  private boolean finished;

  /** Creates a new DriveStraight. */
  public DriveStraight(TankDrivetrain drive, 
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
    drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("Drive Straight Now", "yes");
    boolean hasInitStraight = false;
    PIDController forwardControllerImu = new PIDController(0.001, 0, 0);
    PIDController turnControllerImu = new PIDController(0.01, 0, 0);

    double ticks2Go = 0; // how many encode ticks to move
    double ticks2SlowDown = 0; // when to slow so you don't overshoot

    double drivePower = 0.3;

    double currentTicks = -drive.getTicks();
    double currentMeters = drive.ticksToMeters(currentTicks); 
    ticks2Go = drive.meterToTicks(distanceMeter);//inches2Ticks(distance); // set up encoder stop condition
    ticks2SlowDown = ticks2Go*0.8;//inches2Ticks(distance*0.2); // set up encoder slow down condition
    SmartDashboard.putNumber("Ticks To Go", ticks2Go);
    SmartDashboard.putNumber("Current Ticks", currentTicks);
    
    if (distanceMeter >= 0) {
      double position = drive.getTicks();
      if (position >= ticks2SlowDown) {
        drivePower = 0.15; // cut power prepare to stop
      }
      if (position >= ticks2Go) { // reached desired encoder position
        if (continueMove) {
            drive.setPower(0.1,0.1);
        } 
        else {
            drive.setPower(0, 0);
        }
        SmartDashboard.putString("Finished", "Yes");
        finished = true;
      } 
      else { // move straight
        // drivePower = forwardControllerImu.calculate(currentMeters, distanceMeter);
        SmartDashboard.putNumber("Drive Power", drivePower);
        double rotateToAngleRate = turnControllerImu.calculate(drive.getImu().getHeading(), heading); // calc error correction
        //tankDriveLeftSpeed = (drivePower + rotateToAngleRate);
        //tankDriveRightSpeed = (drivePower - rotateToAngleRate);
        double tankDriveLeftSpeed = NerdyMath.clamp((drivePower + rotateToAngleRate), -maxForwardDriveSpeed, maxForwardDriveSpeed);
        double tankDriveRightSpeed = NerdyMath.clamp((drivePower - rotateToAngleRate), -maxForwardDriveSpeed, maxForwardDriveSpeed);
        drive.setPower(tankDriveLeftSpeed*1.5, tankDriveRightSpeed*1.5);
        SmartDashboard.putNumber("Tank Drive Left Speed", tankDriveLeftSpeed);
        SmartDashboard.putNumber("Tank Drive Right Speed", tankDriveRightSpeed);
        
        SmartDashboard.putString("Finished", "No");
        finished = false;
      } 
    } else {
      drivePower = -0.3;
      double position = drive.getTicks();
      if (position <= ticks2SlowDown)
        drivePower = -0.15; // cut power prepare to stop

      if (position <= ticks2Go) { // reached desired encoder position
        if (continueMove) {
            drive.setPower(-0.1,-0.1);
        } 
        else {
            drive.setPower(0, 0);
        }
        
        SmartDashboard.putString("Finished Back", "Yes");
        finished = true;
      } 
      else { // move straight
        // drivePower = forwardControllerImu.calculate(currentMeters, distanceMeter);
        double rotateToAngleRate = turnControllerImu.calculate(drive.getImu().getHeading(), heading); // calc error correction
        //tankDriveLeftSpeed = (drivePower + rotateToAngleRate);
        //tankDriveRightSpeed = (drivePower - rotateToAngleRate);
        double tankDriveLeftSpeed = NerdyMath.clamp((drivePower + rotateToAngleRate), -maxForwardDriveSpeed, maxForwardDriveSpeed);
        double tankDriveRightSpeed = NerdyMath.clamp((drivePower - rotateToAngleRate), -maxForwardDriveSpeed, maxForwardDriveSpeed);
        drive.setPower(tankDriveLeftSpeed*1.5, tankDriveRightSpeed*1.5);
        SmartDashboard.putNumber("Tank Drive Left Speed", tankDriveLeftSpeed);
        SmartDashboard.putNumber("Tank Drive Right Speed", tankDriveRightSpeed);
        
        
        SmartDashboard.putString("Finished Back", "No");
        finished = false;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
