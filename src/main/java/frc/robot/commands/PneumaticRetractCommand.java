package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatic;

public class PneumaticRetractCommand extends CommandBase {
    private final Pneumatic m_Pneumatic;

    public PneumaticRetractCommand(Pneumatic pcm){
        m_Pneumatic = pcm;
        addRequirements(pcm);
    }

    @Override
    public void execute(){
        m_Pneumatic.Retract();
    }

    

    @Override
    public void end(boolean interrupted){
        m_Pneumatic.stop();
    }
}
