package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;

public class ArmBackwardTempCommand extends CommandBase{
    private final ArmSubsystem m_arm;
    public ArmBackwardTempCommand(ArmSubsystem arm){
        m_arm = arm;
        addRequirements(arm);
    }
    @Override
    public void execute(){
        
        m_arm.operate(-ArmConstants.KArmVelocity);
    }

    @Override
    public void end(boolean interrupted){
        m_arm.stop();
    }
}