// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.Constants.DriveConstants;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.JoystickConstants;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftTurningEncoderPorts,
          DriveConstants.kFrontLeftDriveEncoderReversed,
          DriveConstants.kFrontLeftTurningEncoderReversed);

  private final SwerveModule m_rearLeft =
      new SwerveModule(
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
          DriveConstants.kRearLeftTurningEncoderPorts,
          DriveConstants.kRearLeftDriveEncoderReversed,
          DriveConstants.kRearLeftTurningEncoderReversed);

  private final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightTurningEncoderPorts,
          DriveConstants.kFrontRightDriveEncoderReversed,
          DriveConstants.kFrontRightTurningEncoderReversed);

  private final SwerveModule m_rearRight =
      new SwerveModule(
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
          DriveConstants.kRearRightTurningEncoderPorts,
          DriveConstants.kRearRightDriveEncoderReversed,
          DriveConstants.kRearRightTurningEncoderReversed);

  // The gyro sensor
  //private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        new Rotation2d(),
        //m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_rearLeft.getState(),
        m_frontRight.getState(),
        m_rearRight.getState());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, new Rotation2d());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }
  
    /**
   * Method to drive straight the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   */
  @SuppressWarnings("ParameterName")
  public void drivestraight(double xSpeed, double ySpeed) {
    if(Math.abs(xSpeed) < JoystickConstants.kReadEpsilon){
      xSpeed = 0.0;
    }
    if(Math.abs(ySpeed) < JoystickConstants.kReadEpsilon){
      ySpeed = 0.0;
    }
    double speedMetersPerSecond = Math.sqrt(Math.pow(xSpeed, 2)+Math.pow(ySpeed, 2));
    if(speedMetersPerSecond > DriveConstants.kMaxSpeedMetersPerSecond){
      speedMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecond;
    }
    var angle = new Rotation2d(xSpeed,ySpeed);
    var swerveModuleState = new SwerveModuleState(speedMetersPerSecond,angle);
    
    m_frontLeft.setDesiredState(swerveModuleState);
    m_frontRight.setDesiredState(swerveModuleState);
    m_rearLeft.setDesiredState(swerveModuleState);
    m_rearRight.setDesiredState(swerveModuleState);
  }
  
  public void driveturning(double xSpeed, double ySpeed) {
    if(xSpeed < JoystickConstants.kReadEpsilon){
      xSpeed = 0.0;
    }
    if(ySpeed < JoystickConstants.kReadEpsilon){
      ySpeed = 0.0;
    }
    double speedMetersPerSecond = Math.sqrt(Math.pow(xSpeed, 2)+Math.pow(ySpeed, 2));
    if(speedMetersPerSecond < JoystickConstants.kReadEpsilon){
      return;
    }
    if(speedMetersPerSecond > DriveConstants.kMaxSpeedMetersPerSecond){
      speedMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecond;
    }
    var angle = new Rotation2d(xSpeed,ySpeed);
    var swerveModuleState = new SwerveModuleState(speedMetersPerSecond, angle);
    
    m_frontLeft.setTurnDesiredState(swerveModuleState);
    m_frontRight.setTurnDesiredState(swerveModuleState);
    m_rearLeft.setTurnDesiredState(swerveModuleState);
    m_rearRight.setTurnDesiredState(swerveModuleState);
  }

  public void simpleturningO() {
    
    
    var angle1 = new Rotation2d(1.0,1.0);
    var swerveModuleState1 = new SwerveModuleState(0, angle1);
    var angle2 = new Rotation2d(-1.0,1.0);
    var swerveModuleState2 = new SwerveModuleState(0, angle2);
    
    
    m_frontLeft.setTurnDesiredState(swerveModuleState1);
    m_frontRight.setTurnDesiredState(swerveModuleState2);
    m_rearLeft.setTurnDesiredState(swerveModuleState2);
    m_rearRight.setTurnDesiredState(swerveModuleState1);
  }
  public void simpleturningX() {
    
    
    var angle1 = new Rotation2d(1.0,1.0);
    var swerveModuleState1 = new SwerveModuleState(0, angle1);
    var angle2 = new Rotation2d(-1.0,1.0);
    var swerveModuleState2 = new SwerveModuleState(0, angle2);
    
    
    m_frontLeft.setTurnDesiredState(swerveModuleState2);
    m_frontRight.setTurnDesiredState(swerveModuleState1);
    m_rearLeft.setTurnDesiredState(swerveModuleState1);
    m_rearRight.setTurnDesiredState(swerveModuleState2);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }
  

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    //m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return new Rotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    //return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    return 0.0;
  }
}
