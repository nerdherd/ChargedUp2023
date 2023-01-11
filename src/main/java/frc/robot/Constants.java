// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

 // COMMENT ROBOT IDS INSTEAD OF DELETING
public final class Constants {

  public static class DriveConstants {

    // ============== Gigantor =================
    // public static final int kRightMasterID = 2;
    // public static final int kLeftMasterID = 1;
    // public static final int kRightFollowerID = 3;
    // public static final int kRightFollower2ID = 4;
    // public static final int kLeftFollowerID = 19;
    // public static final int kLeftFollower2ID = 20;
    // public static final double kAngularP = 0;
    // public static final double kAngularD = 0;
    // public static final double kLinearP = 0;
    // public static final double kLinearD = 0;

    // ============== PrototypeBOT =============
    public static final int kRightMasterID = 0;
    public static final int kLeftMasterID = 0;
    public static final double kAngularP = 0;
    public static final double kAngularD = 0;
    public static final double kLinearP = 0;
    public static final double kLinearD = 0;
    public static final int kRightFollowerID = 17;
    public static final int kLeftFollowerID = 31;
    public static final int kRightFollower2ID = 1;
    public static final int kLeftFollower2ID = 18;


    public static final double kDriveAlpha = 0.11765;
    public static final double kDriveOneMinusAlpha = 0.88235;
    
    public static final int kTicksPerFoot = 11738;
    public static final double kErrorBound = 0;

    public static final double kAutoPower = 0;
  }

  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kOperatorJoystickDeadband = 0.05;
  }

  public static class ClawConstants{
    public static final int kPistonForwardID = 2;
    public static final int kPistonReverseID = 3;
    public static final int kPCMPort = 3;
  }

  public static class ArmConstants{
    public static final int kArmID = 0;
    public static final int kArmStow = 0;
    public static final int kArmMiddleNode = 0;
    public static final int kArmTopNode = 0;
    public static final int kArmMotionAcceleration = 0;
    public static final int kArmCruiseVelocity = 0;
    public static final int kArmDeadband = 0;
    public static final int kArmP = 0;
    public static final int kArmI = 0;
    public static final int kArmD = 0;
  }

  public static class VisionConstants{
    public static final double kCameraHeightMeters = Units.inchesToMeters(24);
    public static final double kTargetHeightMeters = Units.inchesToMeters(19.5);
    public static final double kCameraPitchRadians = Units.degreesToRadians(0);
    public static final double kGoalRangeMeters = Units.inchesToMeters(0);
  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kTurningMotorGearRatio = 1 / 21.428; // 150 : 7 : 1 MK4i
    public static final double kMetersPerRevolution = kWheelDiameterMeters * Math.PI;
    public static final double kDriveTicksToMeters = (1 / 2048.0) * kMetersPerRevolution; 
    public static final double kTurningTicksToRad = (1.0 / 4096.0) * 2 * Math.PI;
    public static final double kDriveTicksPer100MsToMetersPerSec = kDriveTicksToMeters * 10;
    public static final double kTurningTicksPer100MsToRadPerSec = kTurningTicksToRad * 10;
    
    public static final double kPTurning = 0.6;
    public static final double kITurning = 0;
    public static final double kDTurning = 0; // TODO: need to tune when on the ground
    
    // TODO: tune PID for drive motor velocity control
    public static final double kPDrive = 0;
    public static final double kIDrive = 0;
    public static final double kDDrive = 0;
  } 


  

  // this needs to get remapped to PS4 controller
  
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;

    public static final double kDeadband = 0.05;    }
        
}
