package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;;

public class ShooterUpperSubsystem extends SubsystemBase{
    private final CANSparkMax m_uppermotor;

    public ShooterUpperSubsystem(int upperID){
        m_uppermotor = new CANSparkMax(ShooterConstants.kUpperMotorPort,MotorType.kBrushless);
    }

    public void UpperShoot(){
        m_uppermotor.set(ShooterConstants.kUpperMotorOutput);
        SmartDashboard.putNumber("ShooterAppliedOutPut", m_uppermotor.getAnalog(Mode.kRelative).getVelocity()*1000);
    }

    public void Stop(){
        m_uppermotor.set(0);
    }
}
