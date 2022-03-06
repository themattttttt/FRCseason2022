package frc.robot.commands;

//import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** A command that will drive the chasis with a fixed angle */
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
    m_speed = DriveConstants.kMaxSpeedMetersPerSecond* Math.sqrt(Math.pow(xSpeed, 2)+Math.pow(ySpeed, 2));
  }

  public DriveCommand(double Speed, DriveSubsystem drive) {
    m_drive = drive;
    addRequirements(m_drive);
    m_speed = DriveConstants.kMaxSpeedMetersPerSecond*Speed;
  }


  @Override
  public void execute(){
    m_drive.drivestraight(m_speed);
    SmartDashboard.putNumber("Set Speed", m_speed);
    SmartDashboard.putNumber("Swerve 1 speed", m_drive.getModuleStates()[0].speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve 2 speed", m_drive.getModuleStates()[1].speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve 3 speed", m_drive.getModuleStates()[2].speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve 4 speed", m_drive.getModuleStates()[3].speedMetersPerSecond);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return m_drive.getModulesAtSpeed(m_speed);
  }
}
