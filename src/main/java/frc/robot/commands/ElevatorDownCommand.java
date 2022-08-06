package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorDownCommand extends CommandBase{
    private final ElevatorSubsystem m_elevator;
    public ElevatorDownCommand(ElevatorSubsystem elevator){
        m_elevator = elevator;
        addRequirements(elevator);
    }
    @Override
    public void execute(){
        m_elevator.operate(ElevatorConstants.kElevatorDownSpeed);

    }

    @Override
    public void end(boolean interrupted){
        m_elevator.operate(0);
    }
}
