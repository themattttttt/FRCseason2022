package frc.robot.commands;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimelightChangeLightCommand extends CommandBase{
    private final LimelightSubsystem m_limelight;

    public LimelightChangeLightCommand(LimelightSubsystem limelight){
        m_limelight = limelight;
        addRequirements(limelight);
    }

    @Override
    public void execute(){
        m_limelight.ChangeLight();
    }

    @Override
    public boolean isFinished(){
        return false;

    }
}