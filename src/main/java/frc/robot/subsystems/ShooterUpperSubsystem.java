package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;;

public class ShooterUpperSubsystem extends SubsystemBase{
    private final CANSparkMax m_uppermotor;

    public ShooterUpperSubsystem(int upperID){
        m_uppermotor = new CANSparkMax(ShooterConstants.kUpperMotorPort,MotorType.kBrushless);
    }

    public void UpperShoot(){
        m_uppermotor.set(ShooterConstants.kUpperMotorOutput);
    }

    public void Stop(){
        m_uppermotor.set(0);
    }
}
