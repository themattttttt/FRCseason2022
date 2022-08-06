package frc.robot.commands;

import frc.robot.subsystems.ArmTempSystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;

public class ArmForwardTempCommand extends CommandBase{
    private final ArmTempSystem m_arm;
    public ArmForwardTempCommand(ArmTempSystem arm){
        m_arm = arm;
        addRequirements(arm);
    }
    @Override
    public void execute(){
        m_arm.operate(ArmConstants.kArmForwardSpeed);
    }

    @Override
    public void end(boolean interrupted){
        m_arm.operate(0);
    }
}