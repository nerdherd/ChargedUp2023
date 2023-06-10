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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    public static final double kDriveAlpha = 0.11765;
    public static final double kDriveOneMinusAlpha = 0.88235;
    public static final double kErrorBound = 0;
  }

  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kTurningMotorGearRatio = 1 / 21.428; // 150 : 7 : 1 MK4i
    public static final double kMetersPerRevolution = kWheelDiameterMeters * Math.PI;
    public static final double kDriveTicksToMeters = (1 / 2048.0) * kMetersPerRevolution; 
    public static final double kAbsoluteTurningTicksToRad = (1.0 / 4096.0) * 2 * Math.PI;
    public static final double kIntegratedTurningTicksToRad = (1.0 / 2048.0) * 2 * Math.PI;
    public static final double kDriveTicksPer100MsToMetersPerSec = kDriveTicksToMeters * 10;
    public static final double kAbsoluteTurningTicksPer100MsToRadPerSec = kAbsoluteTurningTicksToRad * 10;
    public static final double kIntegratedTurningTicksPer100MsToRadPerSec = kIntegratedTurningTicksToRad * 10;

    public static final double kDriveMotorDeadband = 0.02;
    public static final double kTurnMotorDeadband = 0.05;

    public static final double kPTurning = 0.55; // 0.6
    public static final double kITurning = 0;
    public static final double kDTurning = 0.02; 
    
    public static final double kPDrive = 0.13;
    public static final double kIDrive = 0;
    public static final double kDDrive = 0;
    public static final double kFDrive = 0.0469;
  } 

  public static final class SwerveDriveConstants {
    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(21);
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(21);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // TODO: This should be front right not front left
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
    
    public static final double kDodgeDistance = Units.inchesToMeters(12);
    
    public static final Translation2d[] kRotationCenters = new Translation2d[] {
      new Translation2d(kDodgeDistance, -kDodgeDistance),  // FL
      new Translation2d(kDodgeDistance, kDodgeDistance),   // FR
      new Translation2d(-kDodgeDistance, -kDodgeDistance), // BL
      new Translation2d(-kDodgeDistance, kDodgeDistance)   // BR
    };
    
    public static final int[] kLeftRotationCenters = new int[] {0, 1, 3, 2};
    public static final int[] kRightRotationCenters = new int[] {1, 3, 2, 0};

    public static final double kRotationOffset = 0.5 * kTrackWidth;

    public static final int kFRDriveID = 11;
    public static final int kFLDriveID = 21;
    public static final int kBLDriveID = 31;
    public static final int kBRDriveID = 41;

    public static final int kFRTurningID = 12;
    public static final int kFLTurningID = 22;
    public static final int kBLTurningID = 32;
    public static final int kBRTurningID = 42;

    public static final boolean kFRTurningReversed = false; 
    public static final boolean kFLTurningReversed = false; 
    public static final boolean kBLTurningReversed = false; 
    public static final boolean kBRTurningReversed = false; 

    public static final boolean kFRDriveReversed = false;
    public static final boolean kFLDriveReversed = false;     
    public static final boolean kBLDriveReversed = false;      
    public static final boolean kBRDriveReversed = false;

    public static final class MagEncoderConstants {
      public static final int kFRAbsoluteID = 13;
      public static final int kFLAbsoluteID = 23;
      public static final int kBLAbsoluteID = 33;
      public static final int kBRAbsoluteID = 43;

      public static final boolean kFRAbsoluteReversed = false;    
      public static final boolean kFLAbsoluteReversed = false;      
      public static final boolean kBLAbsoluteReversed = false;       
      public static final boolean kBRAbsoluteReversed = false; 

      public static final double kFRAbsoluteOffsetTicks = 2794 + 1024;       
      public static final double kFLAbsoluteOffsetTicks = 2354 + 1024 + 52;         
      public static final double kBLAbsoluteOffsetTicks = 2048 + 1024 + 46;          
      public static final double kBRAbsoluteOffsetTicks = 237 + 1024 + 25;
    }

    public static final class CANCoderConstants {
      public static final int kFRCANCoderID = 14;
      public static final int kFLCANCoderID = 24;
      public static final int kBLCANCoderID = 34;
      public static final int kBRCANCoderID = 44;

      public static final boolean kFRCANCoderReversed = false;    
      public static final boolean kFLCANCoderReversed = false;      
      public static final boolean kBLCANCoderReversed = false;       
      public static final boolean kBRCANCoderReversed = false; 

      public static final double kFRCANCoderOffsetDegrees = 68.379 + 180;       
      public static final double kFLCANCoderOffsetDegrees = 252.949 - 180;         
      public static final double kBLCANCoderOffsetDegrees = 105.293;          
      public static final double kBRCANCoderOffsetDegrees = 180.176 - 180;
    }

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;    
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleMaxAcceleration = 5;
    // THIS CONSTANT HAS TO BE NEGATIVE OTHERWISE THE ROBOT WILL CRASH
    //TODO: Change deceleration with driver feedback, only in small increments (<= -2 is dangerous)
    public static final double kTeleMaxDeceleration = -5; // Russell says he likes 2.5 from sims, but keep at 3 until tested on real robot 

    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
      kPhysicalMaxAngularSpeedRadiansPerSecond * 0.75;
    public static final double kTurnToAngleMaxAngularSpeedRadiansPerSecond 
      = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTurnToBigAngleMaxAngularSpeedRadiansPerSecond = 1.5 * Math.PI;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final double kMinimumMotorOutput = 0.05; // Minimum percent output on the falcons
    
    public static final double kDriveAlpha = 0.11765;
    public static final double kDriveOneMinusAlpha = 0.88235;

    public static final SwerveModuleState[] towModuleStates = 
    new SwerveModuleState[] {
        new SwerveModuleState(0.01, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0.01, Rotation2d.fromDegrees(135)),
        new SwerveModuleState(0.01, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0.01, Rotation2d.fromDegrees(-135))
    };

    public static final double kGravityMPS = 9.80665; 
  }

  public static final class SwerveAutoConstants {
    public static final double kMaxSpeedMetersPerSecond = SwerveDriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
    public static final double kChargeSpeedMetersPerSecond = 0.75 * 2.5;
    public static final double kTwoPieceSpeedMetersPerSecond = 2.8;
    public static final double kMaxAngularSpeedRadiansPerSecond = //
      SwerveDriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = kMaxSpeedMetersPerSecond;
    public static final double kChargeAccelerationMetersPerSecondSquared = 2.5;
    public static final double kTwoPieceAccelerationMetersPerSecondSquared = 2.8;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    public static final double kPXController = SmartDashboard.getNumber("kP X Speed", 0.25);
    public static final double kIXController = SmartDashboard.getNumber("kI X Speed", 0);
    public static final double kDXController = SmartDashboard.getNumber("kD X Speed", 0);
    public static final double kPYController = SmartDashboard.getNumber("kP Y Speed", 0.25);
    public static final double kIYController = SmartDashboard.getNumber("kI Y Speed", 0);
    public static final double kDYController = SmartDashboard.getNumber("kD Y Speed", 0);
    public static final double kPThetaController = SmartDashboard.getNumber("kP Theta Auto", 3.0);
    public static final double kIThetaController = SmartDashboard.getNumber("kI Theta Auto", 0);
    public static final double kDThetaController = SmartDashboard.getNumber("kD Theta Auto", 0);
    public static final double kPTurnToAngle = SmartDashboard.getNumber("kP Theta Teleop", 10.0);
    public static final double kITurnToAngle = SmartDashboard.getNumber("kI Theta Teleop", 0);
    public static final double kDTurnToAngle = SmartDashboard.getNumber("kD Theta Teleop", 0.2);
    public static final double kTurnToAnglePositionToleranceAngle = 5;
    public static final double kTurnToAngleVelocityToleranceAnglesPerSec = 2;
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
      new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularAccelerationRadiansPerSecondSquared);
    public static final double kPBalancingInitial = 4.8;
    public static final double kPBalancing = 2.6; //2.7 worked once //2.37; // 0.4
    public static final double kIBalancing = 0;
    public static final double kDBalancing = 0;
    public static final double kPOneWayBalancing = 2.6; //2.7 worked once //2.37; // 0.4
    public static final double kIOneWayBalancing = 0;
    public static final double kDOneWayBalancing = 0;
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

    public static final double kDeadband = 0.05;
    public static final double kRotationDeadband = 0.1;
  }
}
