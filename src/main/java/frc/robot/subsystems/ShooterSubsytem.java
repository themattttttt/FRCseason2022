package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsytem extends SubsystemBase{
    private final CANSparkMax m_lowermotor;
    private final CANSparkMax m_uppermoter;
    

    public ShooterSubsytem(int lowerID, int upperID){
        m_lowermotor = new CANSparkMax(lowerID,MotorType.kBrushless);
        m_uppermoter = new CANSparkMax(upperID,MotorType.kBrushless);
    }

    public void Shoot(){
        m_lowermotor.set(1);
        m_uppermoter.set(0.2);
    }

    public void Stop(){
        m_lowermotor.set(0);
        m_uppermoter.set(0);
    }
}
