// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 2;
    public static final int kRearLeftDriveMotorPort = 1;
    public static final int kFrontRightDriveMotorPort = 3;
    public static final int kRearRightDriveMotorPort = 4;

    public static final int kFrontLeftTurningMotorPort = 22;
    public static final int kRearLeftTurningMotorPort = 21;
    public static final int kFrontRightTurningMotorPort = 23;
    public static final int kRearRightTurningMotorPort = 24;

    public static final int kFrontLeftTurningEncoderPorts = 52;
    public static final int kRearLeftTurningEncoderPorts = 51;
    public static final int kFrontRightTurningEncoderPorts = 53;
    public static final int kRearRightTurningEncoderPorts = 54;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = true;


    public static final double kFrontLeftTurningEncoderOffset = 81.541;
    public static final double kRearLeftTurningEncoderOffset = 38.463;
    public static final double kFrontRightTurningEncoderOffset = 58.140;
    public static final double kRearRightTurningEncoderOffset = 34.113;


    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = false;

    public static final double kTrackWidth = 0.5;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.7;
    // Distance between front and back wheels on robot

    public static final double kTurnToleranceDeg = 0.0;
    public static final double kTurnRateToleranceDegPerS = 0.0;

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 3000;
  }

  public static final class PIDConfigConstants {
    public static final double kpRearLeftDrive = 0.066;
    public static final double kiRearLeftDrive = 0.00066;
    public static final double kdRearLeftDrive = 1.5;
    public static final double kfRearLeftDrive = 0.046695;

    public static final double kpRearLeftTurn = 1;
    public static final double kiRearLeftTurn = 0.0;
    public static final double kdRearLeftTurn = 0.0;
    public static final double kfRearLeftTurn = 0.39;

    public static final double kpFrontLeftDrive = 0.072;
    public static final double kiFrontLeftDrive = 0.00072;
    public static final double kdFrontLeftDrive = 1.8;
    public static final double kfFrontLeftDrive = 0.046595;

    public static final double kpFrontLeftTurn = 1;
    public static final double kiFrontLeftTurn = 0.0;
    public static final double kdFrontLeftTurn = 0.0;
    public static final double kfFrontLeftTurn = 0.39;

    public static final double kpFrontRightDrive = 0.072;
    public static final double kiFrontRightDrive = 0.00072;
    public static final double kdFrontRightDrive = 1.8;
    public static final double kfFrontRightDrive = 0.046227;

    public static final double kpFrontRightTurn = 1;
    public static final double kiFrontRightTurn = 0.0;
    public static final double kdFrontRightTurn = 0.0;
    public static final double kfFrontRightTurn = 0.39;
    
    public static final double kpRearRightDrive = 0.072;
    public static final double kiRearRightDrive = 0.00072;
    public static final double kdRearRightDrive = 1.8;
    public static final double kfRearRightDrive = 0.046168;

    public static final double kpRearRightTurn = 1;
    public static final double kiRearRightTurn = 0.0;
    public static final double kdRearRightTurn = 0.0;
    public static final double kfRearRightTurn = 0.39;

    
    
    
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeed = 2250;
    public static final double kMaxModuleAngularAcceleration = 2250;

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 0.5;
    public static final double kFModuleTurningController = 0.13;
    public static final double kIModuleTurningController = 0.0;

    public static final double kPModuleDriveController = 0.1;
    public static final double kIModuleDriveController = 0.1;
    public static final double kDModuleDriveController = 0.1;

    public static final double kTurningMax=4096/2*8/3;
    public static final double kPeakOutput=0.8;

    public static final double kTurnToleranceUnit = 50;
    public static final double kTurnToleranceDeg = 3.0;
    public static final double kDriveToleranceUnit = 50;

    public static final double kCANCoderCoefficient = 360.0/4096.0*3.0/8.0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class JoystickConstants{
    public static final double kReadEpsilon = 0.05;
  }
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
  public static final class ArmConstants{
      public static final int kArmMotorPort = 0;
      public static final double kPArmController = 0;
      public static final double kDArmController = 0;
      public static final double kIArmController = 0;
      //public static final double kPArmController = 0;
      public static final double kSVolts = 1;
      public static final double kGVolts = 1;
      public static final double kVVoltSecondPerRad = 0.5;
      public static final double kAVoltSecondSquaredPerRad = 0.1;
  
      public static final double kMaxVelocityRadPerSecond = 3;
      public static final double kMaxAccelerationRadPerSecSquared = 10;
      public static final double kEncoderDistancePerPulse=0;
      public static final double kArmOffsetRads=0;
  }
}
