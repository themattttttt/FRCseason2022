package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterLowerSubsystem;
import frc.robot.subsystems.ShooterUpperSubsystem;

public class ShooterSuckCommand extends CommandBase {
    private final ShooterUpperSubsystem m_upperShooter;
    private final ShooterLowerSubsystem m_lowerShooter;

    public ShooterSuckCommand(ShooterUpperSubsystem upperShooter, ShooterLowerSubsystem lowerShooter){
        m_upperShooter = upperShooter;
        m_lowerShooter = lowerShooter;
        addRequirements(upperShooter);
        addRequirements(lowerShooter);
    }

    @Override
    public void execute(){
        m_lowerShooter.LowerSuck();
        m_upperShooter.UpperSuck();
    }

    @Override
    public void end(boolean interrupted){
        m_lowerShooter.LowerStop();
        m_upperShooter.UpperStop();
    }
}
