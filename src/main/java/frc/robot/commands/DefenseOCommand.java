package frc.robot.commands;

//import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** A command that will turn the robot to the specified angle. */
public class DefenseOCommand extends CommandBase {
  
  
  private final DriveSubsystem m_drive;
  
  public DefenseOCommand( DriveSubsystem drive) {
    m_drive=drive;
    addRequirements(m_drive);
  }
  @Override
  public void execute() {
    SmartDashboard.putBoolean("O reached", isFinished());
    m_drive.simpleturningO();
  }


  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return m_drive.getModulesAtAngle();
  }
}