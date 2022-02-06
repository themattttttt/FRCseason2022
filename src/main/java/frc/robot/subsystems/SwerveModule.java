// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.FilterConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PIDConstant;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;

public class SwerveModule {
  private final TalonFX m_driveMotor;
  private final TalonSRX m_turningMotor;

  private final CANCoder m_turningEncoder;

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, ModuleConstants.kIModuleDriveController , ModuleConstants.kDModuleDriveController);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderPort,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {
    this.m_driveMotor = new TalonFX(driveMotorChannel);
    this.m_turningMotor = new TalonSRX(turningMotorChannel);

    this.m_turningEncoder = new CANCoder(turningEncoderPort);

    //clear sticky faults
    this.m_turningMotor.clearStickyFaults();
    this.m_turningEncoder.clearStickyFaults();
    this.m_driveMotor.clearStickyFaults();

    //For driving motor, use Falcon integrated sensor as PID controller
    //set drving motor profiles
    TalonFXConfiguration talon_configs = new TalonFXConfiguration();
			/* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
			talon_configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
      
			/* config all the settings */
			m_driveMotor.configAllSettings(talon_configs);
      m_driveMotor.setInverted(driveEncoderReversed);


    //use CANCoder to set up feedback for the turning motor
    TalonSRXConfiguration turning_configs = new TalonSRXConfiguration();
    turning_configs.remoteFilter0 = new FilterConfiguration();
    turning_configs.remoteFilter0.remoteSensorDeviceID = turningEncoderPort;
    turning_configs.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    turning_configs.diff0Term=FeedbackDevice.RemoteSensor0;
    turning_configs.sum0Term=FeedbackDevice.RemoteSensor0;
    turning_configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;

    //limit peak output 
    turning_configs.peakOutputForward = PIDConstant.kPeakOutput;
    turning_configs.slot0.kP=PIDConstant.kP;
    turning_configs.slot0.closedLoopPeakOutput = PIDConstant.kPeakOutput;
    turning_configs.slot0.closedLoopPeriod = 10;
    //First, we configure the soft limits on the motor controller 
    //so that they’re enabled and have values for the forward and reverse limits
    turning_configs.forwardSoftLimitEnable=true;
    turning_configs.forwardSoftLimitThreshold=PIDConstant.kTurningMax;
    turning_configs.reverseSoftLimitEnable=true;
    turning_configs.reverseSoftLimitThreshold=-PIDConstant.kTurningMax;
    

    //set configs
    m_turningMotor.configAllSettings(turning_configs);


    
    



    // Set whether turning encoder should be reversed or not
    m_turningMotor.setSensorPhase(true);
    //m_turningMotor.setInverted(turningEncoderReversed);


    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-PIDConstant.kTurningMax, PIDConstant.kTurningMax);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getPosition()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveMotor.getSelectedSensorVelocity(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getPosition(), Math.toDegrees(state.angle.getRadians()));

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(ControlMode.Velocity, driveOutput);
    m_turningMotor.set(ControlMode.Position, turnOutput);
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    m_turningEncoder.setPosition(0);
    m_turningEncoder.clearStickyFaults();
    m_driveMotor.clearStickyFaults();
    m_turningMotor.clearStickyFaults();
  }
}
