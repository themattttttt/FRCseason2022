package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;

public class IntakePositiveCommand extends CommandBase{
    private final IntakeSubsystem m_intake;

    public IntakePositiveCommand(IntakeSubsystem intake){
        m_intake = intake;
        addRequirements(intake);
    }
    @Override
    public void execute(){
        m_intake.operate(IntakeConstants.kPositivePercentageOutput);

    }
    @Override
    public void end(boolean Interrupted){
        m_intake.operate(0);

    }

}