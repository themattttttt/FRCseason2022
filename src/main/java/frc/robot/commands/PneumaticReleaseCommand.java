package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatic;

public class PneumaticReleaseCommand extends CommandBase {
    private final Pneumatic m_Pneumatic;

    public PneumaticReleaseCommand(Pneumatic pcm){
        m_Pneumatic = pcm;
        addRequirements(pcm);
    }

    @Override
    public void execute(){
        m_Pneumatic.Release();
    }


    @Override
    public void end(boolean interrupted){
        m_Pneumatic.stop();
    }
}
