package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterLowerSubsystem;

public class ShooterLowerCommand extends CommandBase {
    private final ShooterLowerSubsystem m_shooter;

    public ShooterLowerCommand(ShooterLowerSubsystem shooter){
        m_shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute(){
        m_shooter.LowerShoot();
    }

    @Override
    public void end(boolean interrupted){
        m_shooter.LowerStop();
    }
}
