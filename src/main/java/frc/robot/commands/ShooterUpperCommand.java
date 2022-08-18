package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterUpperSubsystem;

public class ShooterUpperCommand extends CommandBase {
    private final ShooterUpperSubsystem m_shooter;

    public ShooterUpperCommand(ShooterUpperSubsystem shooter){
        m_shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute(){
        m_shooter.UpperShoot();
    }

    @Override
    public void end(boolean interrupted){
        m_shooter.Stop();
    }
}
