package frc.robot.commands;

//import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** A command that will turn the robot to the specified angle. */
public class DefenseXCommand extends CommandBase {
  private final DriveSubsystem m_drive;
  /**
   * Turns to robot to the specified angle.
   *
   * 
   * @param drive The drive subsystem to use
   */
  public DefenseXCommand( DriveSubsystem drive) {
    m_drive=drive;
    addRequirements(m_drive);
  }
  @Override
  public void execute() {
    //SmartDashboard.putBoolean("X reached", m_drive.getModulesAtAngle());
    m_drive.simpleturningX();
  }


  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return false;
  }
}