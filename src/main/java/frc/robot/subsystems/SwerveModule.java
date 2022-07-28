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
//import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ModuleConstants;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
//import com.ctre.phoenix.motorcontrol.RemoteSensorSource;

public class SwerveModule {
  private final TalonFX m_driveMotor;
  private final TalonSRX m_turningMotor;

  private final CANCoder m_turningEncoder;

  private double m_setAngle = 0.0;
  private double m_optimizedAngle = 0.0;
  private boolean m_Inverted = false;
  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity(), new Rotation2d(Math.toRadians(m_turningEncoder.getPosition())));
  }


  public SwerveModule(
    int driveMotorChannel,
    int turningMotorChannel,
    int turningEncoderPort,
    TalonFXConfiguration drive_config,
    TalonSRXConfiguration turn_config,
    boolean driveEncoderReversed,
    double turningEncoderOffset ) {
    this.m_driveMotor = new TalonFX(driveMotorChannel);
    this.m_turningMotor = new TalonSRX(turningMotorChannel);
    this.m_turningEncoder = new CANCoder(turningEncoderPort);

    //reset CANCoder read offset
    m_turningEncoder.configFeedbackCoefficient(ModuleConstants.kCANCoderCoefficient,"degree",SensorTimeBase.PerSecond);
    double current_reading = m_turningEncoder.getAbsolutePosition();
    double diff = current_reading-turningEncoderOffset;
    //Before the game start, the team should manally calibrate the wheels. Theoratically the diff should be within -60 and 60 in degrees
    /**
     * A full rotation of the wheel results in 8/3 rotations of the ezncoder cylinder (gear ratio 8:3)
     * Absolute reading of the encoder is in range [0,360)
     * A quick example here: if the wheel rotates 1 full circle, the encoder read will change by 8/3 circles, which is +2/3*360 or -1/3*360 in absolute value
     * in short, the increment of absolute value can only be 0, 120 or 240.
     */
    SmartDashboard.putNumber("diff", diff);
    diff = (diff+90) % 45;
    if(diff > 22.5){
      diff = diff-90;
    }
    m_turningEncoder.setPosition(diff);
    


    //config common settings
    this.m_turningMotor.clearStickyFaults();
    this.m_turningEncoder.clearStickyFaults();
    this.m_driveMotor.clearStickyFaults();

    drive_config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    drive_config.diff0Term = FeedbackDevice.IntegratedSensor;
    drive_config.sum0Term = FeedbackDevice.IntegratedSensor;

    drive_config.neutralDeadband = 0.01;
    //config common motor settings
    

    m_driveMotor.configAllSettings(drive_config);
    m_driveMotor.setSensorPhase(true);
    m_driveMotor.setInverted(driveEncoderReversed);
    m_driveMotor.setNeutralMode(NeutralMode.Brake);

    turn_config.remoteFilter0 = new FilterConfiguration();
    turn_config.remoteFilter0.remoteSensorDeviceID = turningEncoderPort;
    turn_config.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    turn_config.diff0Term=FeedbackDevice.RemoteSensor0;
    turn_config.sum0Term=FeedbackDevice.RemoteSensor0;
    turn_config.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;

    //limit peak output 
    turn_config.peakOutputForward = 1;
    turn_config.peakOutputReverse = -1;
    turn_config.nominalOutputForward = 0;
    turn_config.nominalOutputReverse = 0;

    turn_config.slot0.closedLoopPeakOutput = ModuleConstants.kPeakOutput;
    turn_config.slot0.closedLoopPeriod = 1;
    //For integrals, integrate errors out of the zone and accumulate until the max
    turn_config.slot0.allowableClosedloopError = 50;
    turn_config.slot0.integralZone = 300;
    turn_config.slot0.maxIntegralAccumulator = 1000;

    
    //First, we configure the soft limits on the motor controller 
    //so that they’re enabled and have values for the forward and reverse limits
    turn_config.forwardSoftLimitEnable = true;
    turn_config.forwardSoftLimitThreshold = 2*ModuleConstants.kTurningMax;
    turn_config.reverseSoftLimitEnable = true;
    turn_config.reverseSoftLimitThreshold = -2*ModuleConstants.kTurningMax;

    /* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
    turn_config.neutralDeadband = 0.05;

    	/* Set acceleration and vcruise velocity - see documentation */
    //turn_config.motionCruiseVelocity = ModuleConstants.kMaxModuleAngularSpeed;
    //turn_config.motionAcceleration = ModuleConstants.kMaxModuleAngularAcceleration;

    //set configs
    m_turningMotor.configAllSettings(turn_config);

    /* Set relevant frame periods to be at least as fast as periodic rate */
		m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20);
		//m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 20);

    //Set neutral mode to brake to self lock the motor when power on
    m_turningMotor.setNeutralMode(NeutralMode.Brake);

    // Set whether turning encoder should be reversed or not
    m_turningMotor.setSensorPhase(true);
    m_turningMotor.setInverted(false);

    /* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
    }


  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(Math.toRadians(m_turningEncoder.getPosition())));
    m_optimizedAngle = state.angle.getDegrees();
    setTurnDesiredState(state);
    setDesiredSpeed(state.speedMetersPerSecond);
    if (state.speedMetersPerSecond ==0 && !state.angle.equals(desiredState.angle)){
      m_Inverted = true;
    }
    else{
      m_Inverted = false;
    }
  }

  private void setTurnDesiredState(SwerveModuleState desiredState) {
    double turnOutput = getEncoderUnitFromDegrees(desiredState.angle.getDegrees());
    m_turningMotor.set(ControlMode.Position, turnOutput);
  }


  /** get encoder units from degress
   * 
   * @param angle in degrees
   * @return encoder units
   */
  public double getEncoderUnitFromDegrees(double angle)
  {
    return angle/m_turningEncoder.configGetFeedbackCoefficient();
  }

  private void setDesiredSpeed(double driveOutput) {
    if(m_Inverted){
      driveOutput *= -1.0;
    }
    m_driveMotor.set(ControlMode.Velocity, driveOutput);
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    m_turningEncoder.setPosition(0);
    m_turningEncoder.clearStickyFaults();
    m_driveMotor.clearStickyFaults();
    m_turningMotor.clearStickyFaults();
  }

  public void setToZero() {
    SwerveModuleState state = new SwerveModuleState(0.0, new Rotation2d(0.0));
    setTurnDesiredState(state);
  }

  
  /**check if the turing motor reaches the desired angle
   * @param setAngle angle in degress
  */
  public boolean atSetAngle(){
    double diff = Math.abs(m_turningEncoder.getPosition()-m_optimizedAngle);
    return diff < ModuleConstants.kTurnToleranceDeg;
  }
  public void setAngle(double setAngle){
    m_setAngle = setAngle;
  }
  public double getSetAngle(){
    return m_setAngle;
  }
}
