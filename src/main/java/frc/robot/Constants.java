// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    // public static final int kRightMasterID = 0;
    // public static final int kLeftMasterID = 0;
    public static final double kAngularP = 0.1;
    public static final double kAngularD = 0;
    public static final double kLinearP = 0.1;
    public static final double kLinearD = 0;
    // public static final int kRightFollowerID = 17;
    // public static final int kLeftFollowerID = 31;
    public static final int kRightFollower2ID = 1;
    public static final int kLeftFollower2ID = 18;

    // thomas
    public static final int kRightMasterID = 9;
    public static final int kLeftMasterID = 20;
    public static final int kRightFollowerID = 15;
    public static final int kLeftFollowerID = 19;

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
    public static final int kRotatingArmID = 0;
    public static final int kArmID = 19;
    public static final int kArmStow = 0;
    public static final int kArmScore = 0;
    public static final int kArmMotionAcceleration = 0;
    public static final int kArmCruiseVelocity = 0;
    public static final int kArmDeadband = 0;
    public static final int kArmP = 0;
    public static final int kArmI = 0;
    public static final int kArmD = 0;
    public static final int kPistonForwardID = 4;
    public static final int kPistonReverseID = 5;
    public static final double kJoystickMultiplier = 1; 

  }

  public static class VisionConstants{
    public static final double kCameraHeightMeters = Units.inchesToMeters(33.875);
    public static final double kTargetHeightMeters = Units.inchesToMeters(75);
    public static final double kCameraPitchRadians = Units.degreesToRadians(30);
    public static final double kGoalRangeMeters = Units.inchesToMeters(0);
    public static final double kAreaConstant = 0;
  }

  public static final class TankAutoConstants {
    public static final double kPBalancing = 0.3;
    public static final double kDBalancing = 0;

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
    
    public static final double kPTurning = 0.55; // 0.6
    public static final double kITurning = 0;
    public static final double kDTurning = 0.02; 
    
    // TODO: tune PID for drive motor velocity control
    public static final double kPDrive = 0;
    public static final double kIDrive = 0;
    public static final double kDDrive = 0;
  } 

  public static final class SwerveDriveConstants {
    public static final double kTrackWidth = Units.inchesToMeters(21);      // verify
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(21);       // verify
    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    public static final int kFrontLeftDriveMotorPort = 21;
    public static final int kBackLeftDriveMotorPort = 31;
    public static final int kFrontRightDriveMotorPort = 11;
    public static final int kBackRightDriveMotorPort = 41;

    public static final int kFrontLeftTurningMotorPort = 22;
    public static final int kBackLeftTurningMotorPort = 32;
    public static final int kFrontRightTurningMotorPort = 12;
    public static final int kBackRightTurningMotorPort = 42;

    public static final boolean kFrontLeftTurningMotorReversed = false; 
    public static final boolean kBackLeftTurningMotorReversed = false; 
    public static final boolean kFrontRightTurningMotorReversed = false; 
    public static final boolean kBackRightTurningMotorReversed = false; 

    public static final boolean kFrontLeftDriveMotorReversed = false;     
    public static final boolean kBackLeftDriveMotorReversed = false;      
    public static final boolean kFrontRightDriveMotorReversed = false;
    public static final boolean kBackRightDriveMotorReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 23;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 33;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 13;
    public static final int kBackRightDriveAbsoluteEncoderPort = 43;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;      
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;       
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;    
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;      

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 2794;         
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 2354;          
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 237;       
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 2048;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetTicks = 2794;         
    public static final double kBackLeftDriveAbsoluteEncoderOffsetTicks = 2354;          
    public static final double kFrontRightDriveAbsoluteEncoderOffsetTicks = 237; //237        
    public static final double kBackRightDriveAbsoluteEncoderOffsetTicks = 2048;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;    
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
      kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
    public static final double kTurnToAngleMaxAngularSpeedRadiansPerSecond 
      = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    
    public static final double kDriveAlpha = 0.11765;
    public static final double kDriveOneMinusAlpha = 0.88235;

    public static final SwerveModuleState[] towModuleStates = 
    new SwerveModuleState[] {
        new SwerveModuleState(0.01, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0.01, Rotation2d.fromDegrees(135)),
        new SwerveModuleState(0.01, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0.01, Rotation2d.fromDegrees(-135))
    };
  }

  public static final class SwerveAutoConstants {
    public static final double kMaxSpeedMetersPerSecond = SwerveDriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = //
      SwerveDriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3.0;
    public static final double kPTurnToAngle = 10.0;
    public static final double kDTurnToAngle = 0.2;
    public static final double kTurnToAnglePositionToleranceAngle = 5;
    public static final double kTurnToAngleVelocityToleranceAnglesPerSec = 2;
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
      new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularAccelerationRadiansPerSecondSquared);
    public static final double kPBalancingInitial = 0.6;
    public static final double kPBalancing = 0.3;
    public static final double kBalancingDeadbandDegrees = Math.toRadians(2);
    public static final double kBalancingTowPeriod = 0.5;
  }

  // this needs to get remapped to PS4 controller
  
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;

    public static final double kDeadband = 0.05;    }

    public static class BananaConstants {
      public static final double kPIDControllerP = 0.1;
      public static final double kPIDControllerD = 0;
    }
        
}
