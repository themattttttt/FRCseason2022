package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;;

public class ShooterSubsytem extends SubsystemBase{
    private final CANSparkMax m_lowermotor;
    private final CANSparkMax m_uppermoter;
    

    public ShooterSubsytem(int lowerID, int upperID){
        m_lowermotor = new CANSparkMax(ShooterConstants.kLowerMotorPort,MotorType.kBrushless);
        m_uppermoter = new CANSparkMax(ShooterConstants.kUpperMotorPort,MotorType.kBrushless);
    }

    public void Shoot(){
        m_lowermotor.set(ShooterConstants.kLowerMotorOutput);
        m_uppermoter.set(ShooterConstants.kUpperMotorOutput);
    }

    public void out(){
        m_lowermotor.set(ShooterConstants.kLowerMotorOutOutput);
        m_uppermoter.set(0);
    }

    public void Stop(){
        m_lowermotor.set(0);
        m_uppermoter.set(0);
    }
}
