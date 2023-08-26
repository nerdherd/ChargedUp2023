// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.vision.LimelightHelpers;

public class LimelightHelperUser extends SubsystemBase {
  String limelightName = "limelight";

  /** Creates a new ExampleSubsystem. */
  public LimelightHelperUser(String limelightName) {
    this.limelightName = limelightName;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public Pose3d getPose3d() {
    return LimelightHelpers.getLatestResults(limelightName).targetingResults.getBotPose3d();
  }

  public double getX() {
    return getPose3d().getX();
  }

  public double getY() {
    return getPose3d().getY();
  }

  public void reportToSmartDashboard() {
    SmartDashboard.putNumber("Vision Robot X", getX());
    SmartDashboard.putNumber("Vision Robot Y", getY());
  }
}
