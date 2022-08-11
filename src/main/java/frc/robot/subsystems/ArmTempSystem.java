package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class ArmTempSystem extends SubsystemBase{
    private final TalonFX m_motor = new TalonFX(ArmConstants.kArmMotorPort);
    private final TalonFX m_motor2 = new TalonFX(ArmConstants.kArmMotorPort2);
    TalonFXConfiguration drive_config;

    public ArmTempSystem(){
        this.drive_config = getDrivePIDconfig(
            ArmConstants.kPArmController,
            ArmConstants.kIArmController,
            ArmConstants.kDArmController,
            ArmConstants.kFArmController);
    
        this.m_motor.clearStickyFaults();
        this.m_motor2.clearStickyFaults();

        drive_config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        drive_config.diff0Term = FeedbackDevice.IntegratedSensor;
        drive_config.sum0Term = FeedbackDevice.IntegratedSensor;

        drive_config.neutralDeadband = 0.01;

        m_motor.configAllSettings(drive_config);
        m_motor.setSensorPhase(true);
        m_motor.setNeutralMode(NeutralMode.Brake);
        m_motor.configMotionCruiseVelocity(ArmConstants.KArmVelocity);
        m_motor.configMotionAcceleration(ArmConstants.kArmAcceleration);
        m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20);
        m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 20);

        m_motor2.configAllSettings(drive_config);
        m_motor2.setSensorPhase(true);
        m_motor2.setNeutralMode(NeutralMode.Brake);
        m_motor2.configMotionCruiseVelocity(ArmConstants.KArmVelocity);
        m_motor2.configMotionAcceleration(ArmConstants.kArmAcceleration);
        m_motor2.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20);
        m_motor2.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 20);
    }

    public static TalonFXConfiguration getDrivePIDconfig(double kP, double kI, double kD, double kF){
        TalonFXConfiguration talon_config = new TalonFXConfiguration();
        talon_config.slot0.kP = kP;
        talon_config.slot0.kI=kI;
        talon_config.slot0.kD=kD;
        talon_config.slot0.kF=kF;
        return talon_config;
      }

    public void operate(double pos) {
        m_motor.setSelectedSensorPosition(0);
        m_motor2.setSelectedSensorPosition(0);
        m_motor.set(TalonFXControlMode.MotionMagic,pos);
        m_motor2.follow(m_motor);
    }
}
