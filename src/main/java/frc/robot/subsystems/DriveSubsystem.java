// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import frc.robot.Constants.PIDConfigConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.Constants.DriveConstants;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.ModuleConstants;
import java.util.List;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftTurningEncoderPorts,
          getDrivePIDconfig(PIDConfigConstants.kpFrontLeftDrive, 
                       PIDConfigConstants.kiFrontLeftDrive, 
                       PIDConfigConstants.kdFrontLeftDrive, 
                       PIDConfigConstants.kfFrontLeftDrive),
          getTurnPIDconfig(PIDConfigConstants.kpFrontLeftTurn, 
                       PIDConfigConstants.kiFrontLeftTurn, 
                       PIDConfigConstants.kdFrontLeftTurn, 
                       PIDConfigConstants.kfFrontLeftTurn),
          DriveConstants.kFrontLeftDriveEncoderReversed,
          DriveConstants.kFrontLeftTurningEncoderOffset
          );

  private final SwerveModule m_rearLeft =
      new SwerveModule(
        DriveConstants.kRearLeftDriveMotorPort,
        DriveConstants.kRearLeftTurningMotorPort,
        DriveConstants.kRearLeftTurningEncoderPorts,
        getDrivePIDconfig(PIDConfigConstants.kpRearLeftDrive, 
                     PIDConfigConstants.kiRearLeftDrive, 
                     PIDConfigConstants.kdRearLeftDrive, 
                     PIDConfigConstants.kfRearLeftDrive),
        getTurnPIDconfig(PIDConfigConstants.kpRearLeftTurn, 
                     PIDConfigConstants.kiRearLeftTurn, 
                     PIDConfigConstants.kdRearLeftTurn, 
                     PIDConfigConstants.kfRearLeftTurn),
        DriveConstants.kRearLeftDriveEncoderReversed,
        DriveConstants.kRearLeftTurningEncoderOffset
      );

  private final SwerveModule m_frontRight =
      new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightTurningEncoderPorts,
        getDrivePIDconfig(PIDConfigConstants.kpFrontRightDrive, 
                     PIDConfigConstants.kiFrontRightDrive, 
                     PIDConfigConstants.kdFrontRightDrive, 
                     PIDConfigConstants.kfFrontRightDrive),
        getTurnPIDconfig(PIDConfigConstants.kpFrontRightTurn, 
                     PIDConfigConstants.kiFrontRightTurn, 
                     PIDConfigConstants.kdFrontRightTurn, 
                     PIDConfigConstants.kfFrontRightTurn),
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderOffset
      );

  private final SwerveModule m_rearRight =
      new SwerveModule(
        DriveConstants.kRearRightDriveMotorPort,
        DriveConstants.kRearRightTurningMotorPort,
        DriveConstants.kRearRightTurningEncoderPorts,
        getDrivePIDconfig(PIDConfigConstants.kpRearRightDrive, 
                     PIDConfigConstants.kiRearRightDrive, 
                     PIDConfigConstants.kdRearRightDrive, 
                     PIDConfigConstants.kfRearRightDrive),
        getTurnPIDconfig(PIDConfigConstants.kpRearRightTurn, 
                     PIDConfigConstants.kiRearRightTurn, 
                     PIDConfigConstants.kdRearRightTurn, 
                     PIDConfigConstants.kfRearRightTurn),
        DriveConstants.kRearRightDriveEncoderReversed,
        DriveConstants.kRearRightTurningEncoderOffset
        );
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

  public void setCurrentAngle(double angle){
    m_frontLeft.setAngle(angle);
    m_frontRight.setAngle(angle);
    m_rearLeft.setAngle(angle);
    m_rearRight.setAngle(angle);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, new Rotation2d());
    setCurrentAngle(0.0);
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
    if(Math.abs(xSpeed) < JoystickConstants.kReadEpsilon){
      xSpeed = 0.0;
    }
    if(Math.abs(ySpeed) < JoystickConstants.kReadEpsilon){
      ySpeed = 0.0;
    }
    xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    //m_rearLeft.setTurnDesiredState(swerveModuleStates[2]);
    //m_rearRight.setTurnDesiredState(swerveModuleStates[3]);
    //TalonFX master_left_falcon = m_frontLeft.getDriveMotor();
    //TalonFX master_right_falcon = m_frontRight.getDriveMotor();
    //m_rearLeft.FollowDrive(master_left_falcon);
    //m_rearRight.FollowDrive(master_right_falcon);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
    
    //SmartDashboard.putNumber("1 speed", swerveModuleStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("1 kf", PIDConfigConstants.kfRearLeftDrive);
    //SmartDashboard.putNumber("m_frontLeft_angle", swerveModuleStates[0].angle.getDegrees());
    //SmartDashboard.putNumber("2 speed", swerveModuleStates[1].speedMetersPerSecond);
    SmartDashboard.putNumber("2 kf", PIDConfigConstants.kfFrontLeftDrive);
    //SmartDashboard.putNumber("m_frontRight_angle", swerveModuleStates[1].angle.getDegrees());
    //SmartDashboard.putNumber("3 speed", swerveModuleStates[2].speedMetersPerSecond);
    SmartDashboard.putNumber("3 kf", PIDConfigConstants.kfFrontRightDrive);
   // SmartDashboard.putNumber("m_rearLeft_angle", swerveModuleStates[2].angle.getDegrees());
    ///SmartDashboard.putNumber("4 speed", swerveModuleStates[3].speedMetersPerSecond);
    SmartDashboard.putNumber("4 kf", PIDConfigConstants.kfRearRightDrive);
    //SmartDashboard.putNumber("m_rearRight_angle", swerveModuleStates[3].angle.getDegrees());
    SmartDashboard.putNumber("1 Sensor Position", m_rearLeft.getDriveSensorPosition());
    SmartDashboard.putNumber("2 Sensor Position", m_frontLeft.getDriveSensorPosition());
    SmartDashboard.putNumber("3 Sensor Position", m_rearLeft.getDriveSensorPosition());
    SmartDashboard.putNumber("4 Sensor Position", m_rearRight.getDriveSensorPosition());
  }
  

  public void simpleturningO() {
    var angle1 = new Rotation2d(1.0,1.0);
    var swerveModuleState1 = new SwerveModuleState(0, angle1);
    var angle2 = new Rotation2d(-1.0,1.0);
    var swerveModuleState2 = new SwerveModuleState(0, angle2);
    
    m_frontLeft.setDesiredState(swerveModuleState1);
    m_frontLeft.setAngle(angle1.getDegrees());
    m_frontRight.setDesiredState(swerveModuleState2);
    m_frontLeft.setAngle(angle2.getDegrees());
    m_rearLeft.setDesiredState(swerveModuleState2);
    m_frontLeft.setAngle(angle2.getDegrees());
    m_rearRight.setDesiredState(swerveModuleState1);
    m_frontLeft.setAngle(angle1.getDegrees());
  }
  public void simpleturningX() {
    var angle1 = new Rotation2d(1.0,1.0);
    var swerveModuleState1 = new SwerveModuleState(0, angle1);
    var angle2 = new Rotation2d(-1.0,1.0);
    var swerveModuleState2 = new SwerveModuleState(0, angle2);
  
    m_frontLeft.setDesiredState(swerveModuleState2);
    m_frontLeft.setAngle(angle2.getDegrees());
    m_frontRight.setDesiredState(swerveModuleState1);
    m_frontLeft.setAngle(angle1.getDegrees());
    m_rearLeft.setDesiredState(swerveModuleState1);
    m_frontLeft.setAngle(angle1.getDegrees());
    m_rearRight.setDesiredState(swerveModuleState2);
    m_frontLeft.setAngle(angle2.getDegrees());
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

  /**Check if all four modules reaches desired angle
   * 
   * @param setAngle in degrees
   * @return
   */
  public boolean getModulesAtAngle(){
    
    //SmartDashboard.putBoolean("Swerve 1 Angle State", m_rearLeft.atSetAngle());
    //SmartDashboard.putBoolean("Swerve 2 Angle State", m_frontLeft.atSetAngle());
    //SmartDashboard.putBoolean("Swerve 3 Angle State", m_frontRight.atSetAngle());
    //SmartDashboard.putBoolean("Swerve 4 Angle State", m_rearRight.atSetAngle());
    
    if (!m_frontLeft.atSetAngle()){
      return false;
    }
    else if (!m_frontRight.atSetAngle()){
      return false;
    }
    else if (!m_rearLeft.atSetAngle()){
      return false;
    }
    else if (!m_rearRight.atSetAngle()){
      return false;
    }
    else{
      return true;
    }
  }

  /**
   * 
   * @param velocity
   * @return
   */
  public boolean getModulesAtSpeed(double velocity){
    if(Math.abs(m_rearRight.getState().speedMetersPerSecond-velocity)>ModuleConstants.kDriveToleranceUnit){
      return false;
    }
    else if(Math.abs(m_rearLeft.getState().speedMetersPerSecond-velocity)>ModuleConstants.kDriveToleranceUnit){
      return false;
    }
    else if(Math.abs(m_frontLeft.getState().speedMetersPerSecond-velocity)>ModuleConstants.kDriveToleranceUnit){
      return false;
    }
    else if(Math.abs(m_frontRight.getState().speedMetersPerSecond-velocity)>ModuleConstants.kDriveToleranceUnit){
      return false;
    }
    else{
      return true;
    }
  }

  public static TalonFXConfiguration getDrivePIDconfig(double kP, double kI, double kD, double kF){
    TalonFXConfiguration talon_config = new TalonFXConfiguration();
    talon_config.slot0.kP = kP;
    talon_config.slot0.kI=kI;
    talon_config.slot0.kD=kD;
    talon_config.slot0.kF=kF;
    return talon_config;
  }
  
  public static TalonSRXConfiguration getTurnPIDconfig(double kP, double kI, double kD, double kF){
    TalonSRXConfiguration turning_config = new TalonSRXConfiguration();
    turning_config.slot0.kP = kP;
    turning_config.slot0.kI=kI;
    turning_config.slot0.kD=kD;
    turning_config.slot0.kF=kF;
    return turning_config;
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] stateArray = {m_rearLeft.getState(), m_frontLeft.getState(),m_frontRight.getState(),m_rearRight.getState()};
    return stateArray;
  }

  public void setAllToZero(){
    m_frontLeft.setToZero();
    m_frontRight.setToZero();
    m_rearRight.setToZero();
    m_rearLeft.setToZero();
  }
}
