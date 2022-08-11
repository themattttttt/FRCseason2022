package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ElevatorSubsystem extends SubsystemBase{
    private final TalonFX m_motor;
    TalonFXConfiguration drive_config;

    public 
    ElevatorSubsystem(){
        this.m_motor = new TalonFX(ElevatorConstants.kElevatorMotorPort);
        this.drive_config = getDrivePIDconfig(
            ElevatorConstants.kpElevator,
            ElevatorConstants.kiElevator,
            ElevatorConstants.kdElevator,
            ElevatorConstants.kfElevator);
    
        this.m_motor.clearStickyFaults();

        drive_config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        drive_config.diff0Term = FeedbackDevice.IntegratedSensor;
        drive_config.sum0Term = FeedbackDevice.IntegratedSensor;

        drive_config.neutralDeadband = 0.01;

        m_motor.configAllSettings(drive_config);
        m_motor.setSensorPhase(true);
        m_motor.setNeutralMode(NeutralMode.Brake);

    }

    public static TalonFXConfiguration getDrivePIDconfig(double kP, double kI, double kD, double kF){
        TalonFXConfiguration talon_config = new TalonFXConfiguration();
        talon_config.slot0.kP = kP;
        talon_config.slot0.kI=kI;
        talon_config.slot0.kD=kD;
        talon_config.slot0.kF=kF;
        return talon_config;
      }

    public void operate(int speed) {
        m_motor.set(ControlMode.Velocity,speed);
    }
}
