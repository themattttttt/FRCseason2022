package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
public class Pneumatic extends SubsystemBase {
    //private final CANSparkMax m_lowermotor;
    //private final CANSparkMax m_uppermoter;
    private Compressor m_pcmCompressor;
    private DoubleSolenoid m_pcmSolenoid;

    

    public Pneumatic(int PCM_id){
        m_pcmCompressor = new Compressor(PCM_id, PneumaticsModuleType.CTREPCM);
        m_pcmCompressor.enableDigital();
        m_pcmSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
        //m_pcmCompressor.disable();
        
        boolean enabled = m_pcmCompressor.enabled();
        boolean pressureSwitch = m_pcmCompressor.getPressureSwitchValue();
        String configtype = m_pcmCompressor.getConfigType().name();
        double current = m_pcmCompressor.getCurrent();

        //SmartDashboard.putString("type", configtype);
        //SmartDashboard.putBoolean("enabled", enabled);
        //SmartDashboard.putBoolean("ps found",pressureSwitch);
        //SmartDashboard.putNumber("current", current);

    }
    public void Release()
    {
        m_pcmSolenoid.set(kForward);
    }
    public void Retract()
    {
        m_pcmSolenoid.set(kReverse);
    }
    public void stop()
    {
        m_pcmSolenoid.set(kOff);
    }
}

