package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;;

public class ShooterSubsytem extends SubsystemBase{
    private final CANSparkMax m_lowermotor;
    private final CANSparkMax m_uppermotor;
    private final SparkMaxPIDController m_pidController;
    

    public ShooterSubsytem(int lowerID, int upperID){
        m_lowermotor = new CANSparkMax(ShooterConstants.kLowerMotorPort,MotorType.kBrushless);
        m_uppermotor = new CANSparkMax(ShooterConstants.kUpperMotorPort,MotorType.kBrushless);
        m_pidController = m_lowermotor.getPIDController();

        m_pidController.setP(ShooterConstants.kpShooter);
        m_pidController.setD(ShooterConstants.kdShooter);
        m_pidController.setFF(ShooterConstants.kfShooter);
        m_pidController.setOutputRange(ShooterConstants.kMinOutputShooter, ShooterConstants.kMaxOutputShooter);
        
    }

    public void Shoot(){
        m_pidController.setReference(ShooterConstants.kRPM, CANSparkMax.ControlType.kVelocity);
        m_uppermotor.set(ShooterConstants.kUpperMotorOutput);
    }

    public void out(){
        m_pidController.setReference(ShooterConstants.kRPM, CANSparkMax.ControlType.kVelocity);
        m_uppermotor.set(0);
    }

    public void Stop(){
        m_lowermotor.set(0);
        m_uppermotor.set(0);
    }
}
