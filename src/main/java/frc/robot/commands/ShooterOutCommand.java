package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsytem;

public class ShooterOutCommand extends CommandBase {
    private final ShooterSubsytem m_shooter;

    public ShooterOutCommand(ShooterSubsytem shooter){
        m_shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute(){
        m_shooter.out();
    }

    @Override
    public void end(boolean interrupted){
        m_shooter.Stop();
    }
}
