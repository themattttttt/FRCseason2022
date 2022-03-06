package frc.robot.commands;

//import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


/** A command that will turn the robot to the specified angle. */
public class DriveCommand extends CommandBase {
  private final DriveSubsystem m_drive;
  private final double m_speed;
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetSpeed The speed of motor
   * @param drive The drive subsystem to use
   */
  public DriveCommand(double xSpeed,double ySpeed, DriveSubsystem drive) {
    m_drive = drive;
    addRequirements(m_drive);
    m_speed = Math.sqrt(Math.pow(xSpeed, 2)+Math.pow(ySpeed, 2));
  }

  @Override
  public void execute(){
    m_drive.drivestraight(m_speed);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return m_drive.getModulesAtSpeed(m_speed);
  }
}
