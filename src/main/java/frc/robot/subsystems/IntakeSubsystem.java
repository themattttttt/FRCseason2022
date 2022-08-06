package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    private final TalonSRX m_motor;

    public IntakeSubsystem(){
        this.m_motor = new TalonSRX(IntakeConstants.kIntakeMotorPort);
    }

    public void operate(double output){
        m_motor.set(ControlMode.PercentOutput, output);
    }


}
