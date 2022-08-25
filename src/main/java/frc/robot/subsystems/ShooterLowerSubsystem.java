package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;;

public class ShooterLowerSubsystem extends SubsystemBase{
    private final CANSparkMax m_lowermotor;
    private final SparkMaxPIDController m_pidController;
    
    public ShooterLowerSubsystem(int lowerID){
        m_lowermotor = new CANSparkMax(ShooterConstants.kLowerMotorPort,MotorType.kBrushless);
        m_pidController = m_lowermotor.getPIDController();

        m_pidController.setP(ShooterConstants.kpShooter);
        m_pidController.setD(ShooterConstants.kdShooter);
        m_pidController.setFF(ShooterConstants.kfShooter);
        m_pidController.setOutputRange(ShooterConstants.kMinOutputShooter, ShooterConstants.kMaxOutputShooter);
    }


    public void LowerShoot(){
        m_pidController.setReference(ShooterConstants.kRPM, CANSparkMax.ControlType.kVelocity);
    }

    public void LowerSuck(){
        m_pidController.setReference(ShooterConstants.kRPMSuck, CANSparkMax.ControlType.kVelocity);
    }

    public void LowerStop(){
        m_lowermotor.set(0);
    }
    
}
