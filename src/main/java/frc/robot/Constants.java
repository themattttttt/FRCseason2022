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


    public static final double kFrontLeftTurningEncoderOffset = 21.413 ;
    public static final double kRearLeftTurningEncoderOffset = 47.362;
    public static final double kFrontRightTurningEncoderOffset = 58.140;
    public static final double kRearRightTurningEncoderOffset = 23.269;


    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = false;

    public static final double kTrackWidth = 0.5;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.5;
    // Distance between front and back wheels on robot

    public static final double kTurnToleranceDeg = 0.0;
    public static final double kTurnRateToleranceDegPerS = 0.0;

    public static final double kAngularSpeed = 12000;

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 12000;
  }

  public static final class PIDConfigConstants {
    public static final double kpRearLeftDrive = 0.05;
    public static final double kiRearLeftDrive = 0;
    public static final double kdRearLeftDrive = 1;
    public static final double kfRearLeftDrive = 0.046695;

    public static final double kpRearLeftTurn = 0.3;
    public static final double kiRearLeftTurn = 0.05;
    public static final double kdRearLeftTurn = 0.0;
    public static final double kfRearLeftTurn = 0.43;

    public static final double kpFrontLeftDrive = 0.05;
    public static final double kiFrontLeftDrive = 0;
    public static final double kdFrontLeftDrive = 1;
    public static final double kfFrontLeftDrive = 0.046;

    public static final double kpFrontLeftTurn = 0.255;
    public static final double kiFrontLeftTurn = 0.08;
    public static final double kdFrontLeftTurn = 1.5;
    public static final double kfFrontLeftTurn = 0.555;

    public static final double kpFrontRightDrive = 0.1;
    public static final double kiFrontRightDrive = 0;
    public static final double kdFrontRightDrive = 3;
    public static final double kfFrontRightDrive = 0.06;

    public static final double kpFrontRightTurn = 0.35;
    public static final double kiFrontRightTurn = 0.05;
    public static final double kdFrontRightTurn = 0.0;
    public static final double kfFrontRightTurn = 0.5;
    
    public static final double kpRearRightDrive = 0.075;
    public static final double kiRearRightDrive = 0;
    public static final double kdRearRightDrive = 2;
    public static final double kfRearRightDrive = 0.055;

    public static final double kpRearRightTurn = 0.6;
    public static final double kiRearRightTurn = 0.05;
    public static final double kdRearRightTurn = 2;
    public static final double kfRearRightTurn = 0.6;

    
    
    
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

    public static final double kTurningMax=5461; //4096*8/3/2.0(180 degrees)
    public static final double kPeakOutput=0.8;

    public static final double kTurnToleranceUnit = 50;
    public static final double kTurnToleranceDeg = 3.0;
    public static final double kDriveToleranceUnit = 50;

    public static final double kCANCoderCoefficient = 360.0/4096.0*3.0/8.0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperateControllerPort = 1;
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
      public static final int kArmMotorPort = 5;
      public static final int kArmMotorPort2 = 7;
      public static final double kPArmController = 0.035;
      public static final double kDArmController = 0.0;
      public static final double kIArmController = 0;
      public static final double kFArmController = 0.05;


      public static final double kArmBackwardPos = 300;
      public static final double kArmForwardPos = -300;
      public static final double KArmVelocity = -1800;
      public static final double kArmAcceleration = 1500;

      public static final double kArmForwardLimit = -30000;
      public static final double kArmBackwardLimit = 16500;

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

  public static final class IntakeConstants{
    public static final int kIntakeMotorPort = 25;
    public static final double kPositivePercentageOutput = 0.5;
    public static final double kNegativePercentageOutput = -0.5;
  }

  public static final class ShooterConstants{
    public static final int kUpperMotorPort = 32;
    public static final int kLowerMotorPort = 31;
    public static final int  kRPM = 1000;
    public static final int kRPMSuck = -1000;
    public static final double kpShooter = 0.00035;
    public static final double kdShooter = 0.005;
    public static final double kfShooter = 0.0002;
    public static final double kMaxOutputShooter = 1.0;
    public static final double kMinOutputShooter = -1.0;
    public static final double kUpperMotorOutput = 1.0;
    public static final double kUpperSuckMotorOutput = -0.5;

  }

  public static final class ElevatorConstants{
      public static final int kElevatorMotorPort = 6;
      public static final double kpElevator = 0.05115;
      public static final double kiElevator = 0;
      public static final double kdElevator = 2;
      public static final double kfElevator = 0.043;
      public static final int kElevatorUpSpeed = -10000;
      public static final int kElevatorDownSpeed = 10000;
  }

  public static final class OperateConstants{
    public static final int kLowerShooterChannel = 31;
    public static final int kUpperShooterChannel = 32;
  }

  public static final class LimelightConstants{
    public static final double kHeightOfTheTarget = 270.0;
    public static final double kHeightOfTheLimelight = 70.0;
    public static final double kAngleOfTheLimilight = 42.4;
    public static final double kDistanceWithTargetMax = 130.0;
    public static final double kDistanceWithTargetMin = 110.0;
    public static final double kAngleMax = 5.0;
    public static final double kAngleMin = -5.0;
    public static final double kLimelightFactor = 150.0;

  }
}
