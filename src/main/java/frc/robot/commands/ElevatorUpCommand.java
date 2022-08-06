package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorUpCommand extends CommandBase{
    private final ElevatorSubsystem m_elevator;
    public ElevatorUpCommand(ElevatorSubsystem elevator){
        m_elevator = elevator;
        addRequirements(elevator);
    }
    @Override
    public void execute(){
        m_elevator.operate(ElevatorConstants.kElevatorUpSpeed);
    }

    @Override
    public void end(boolean interrupted){
        m_elevator.operate(0);
    }
}
