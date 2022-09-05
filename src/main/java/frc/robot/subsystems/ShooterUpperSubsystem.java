package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;;

public class ShooterUpperSubsystem extends SubsystemBase{
    private final CANSparkMax m_uppermotorRight;
    private final CANSparkMax m_uppermotorLeft;

    public ShooterUpperSubsystem(int upperRightID,int upperLeftID){
        m_uppermotorRight = new CANSparkMax(upperRightID,MotorType.kBrushless);
        m_uppermotorLeft = new CANSparkMax(upperLeftID,MotorType.kBrushless);
        m_uppermotorLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_uppermotorRight.setIdleMode(CANSparkMax.IdleMode.kCoast);

    }

    public void UpperShoot(){
        m_uppermotorRight.set(ShooterConstants.kUpperMotorOutput);
        m_uppermotorLeft.follow(m_uppermotorRight, true);
        SmartDashboard.putNumber("ShooterAppliedOutPut", m_uppermotorRight.getAnalog(Mode.kRelative).getVelocity()*1000);
    }

    public void UpperSuck(){
        m_uppermotorRight.set(ShooterConstants.kUpperSuckMotorOutput);
        m_uppermotorLeft.follow(m_uppermotorRight, true);
    }
    
    public void UpperStop(){
        m_uppermotorRight.set(0);
        m_uppermotorLeft.set(0);
    }
}
