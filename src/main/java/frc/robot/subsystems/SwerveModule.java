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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ModuleConstants;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;

public class SwerveModule {
  private final TalonFX m_driveMotor;
  private final TalonSRX m_turningMotor;

  private final CANCoder m_turningEncoder;

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

    //set up coefficent to make sure the encoder returns a reading of actual degrees
    double turning_coefficient = m_turningEncoder.configGetFeedbackCoefficient();


    //For driving motor, use Falcon integrated sensor as PID controller
    //set drving motor profiles
    TalonFXConfiguration talon_configs = new TalonFXConfiguration();
		/* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
		talon_configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    talon_configs.diff0Term = FeedbackDevice.IntegratedSensor;
    talon_configs.sum0Term = FeedbackDevice.IntegratedSensor;
      
		/* config all the settings */
		m_driveMotor.configAllSettings(talon_configs);
    m_driveMotor.setSensorPhase(true);
    m_driveMotor.setInverted(driveEncoderReversed);
    m_driveMotor.setNeutralMode(NeutralMode.Brake);


    //use CANCoder to set up feedback for the turning motor
    TalonSRXConfiguration turning_configs = new TalonSRXConfiguration();
    turning_configs.remoteFilter0 = new FilterConfiguration();
    turning_configs.remoteFilter0.remoteSensorDeviceID = turningEncoderPort;
    turning_configs.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    turning_configs.diff0Term=FeedbackDevice.RemoteSensor0;
    turning_configs.sum0Term=FeedbackDevice.RemoteSensor0;
    turning_configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;

    //limit peak output 
    turning_configs.peakOutputForward = 1;
    turning_configs.peakOutputReverse = -1;
    turning_configs.nominalOutputForward = 0;
    turning_configs.nominalOutputReverse = 0;

    //set turning motor PID 
    turning_configs.slot0.kP = ModuleConstants.kPModuleTurningController;
    turning_configs.slot0.kF = ModuleConstants.kFModuleTurningController;
    turning_configs.slot0.kI = ModuleConstants.kIModuleTurningController;
    turning_configs.slot0.closedLoopPeakOutput = ModuleConstants.kPeakOutput;
    turning_configs.slot0.closedLoopPeriod = 1;
    //For integrals, integrate errors out of the zone and accumulate until the max
    turning_configs.slot0.integralZone = 100;
    turning_configs.slot0.maxIntegralAccumulator = 1000;

    //First, we configure the soft limits on the motor controller 
    //so that they’re enabled and have values for the forward and reverse limits
    turning_configs.forwardSoftLimitEnable = true;
    turning_configs.forwardSoftLimitThreshold = 2*ModuleConstants.kTurningMax;
    turning_configs.reverseSoftLimitEnable = true;
    turning_configs.reverseSoftLimitThreshold = -2*ModuleConstants.kTurningMax;

    /* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
    turning_configs.neutralDeadband = 0.05;

    	/* Set acceleration and vcruise velocity - see documentation */
    turning_configs.motionCruiseVelocity = ModuleConstants.kMaxModuleAngularSpeed;
    turning_configs.motionAcceleration = ModuleConstants.kMaxModuleAngularAcceleration;

    //set configs
    m_turningMotor.configAllSettings(turning_configs);

    /* Set relevant frame periods to be at least as fast as periodic rate */
		m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20);
		m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 20);

    //Set neutral mode to brake to self lock the motor when power on
    m_turningMotor.setNeutralMode(NeutralMode.Brake);
    

    // Set whether turning encoder should be reversed or not
    m_turningMotor.setSensorPhase(true);
    //m_turningMotor.setInverted(turningEncoderReversed);


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

    // Calculate the turning motor output from the turning PID controller.
    double driveOutput = state.speedMetersPerSecond;
    //use raw readings and multiply by the ratio to get the actual turnning of the gear
    double turnOutput = state.angle.getDegrees()/m_turningEncoder.configGetFeedbackCoefficient()*8/3;
    //m_driveMotor.set(ControlMode.Velocity, driveOutput);
    m_turningMotor.set(ControlMode.MotionMagic, turnOutput);
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    m_turningEncoder.setPosition(0);
    m_turningEncoder.clearStickyFaults();
    m_driveMotor.clearStickyFaults();
    m_turningMotor.clearStickyFaults();
  }
}
