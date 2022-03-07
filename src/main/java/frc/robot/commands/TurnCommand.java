package frc.robot.commands;

//mport edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** A command that will turn the robot to the specified angle. */
public class TurnCommand extends CommandBase {
  // The subsystem the command runs on
  private final DriveSubsystem m_drive;
  private final double m_xSpeed;
  private final double m_ySpeed;
  private final double m_degree;
  /**
   * Turns to robot to the specified angle.
   *
   * @param xSpeed  x component or cosine value
   * @param ySpeed y component or sine value
   * @param drive The drive subsystem to use
   */
  public TurnCommand(double xSpeed, double ySpeed, DriveSubsystem drive) {
    m_drive = drive;
    addRequirements(m_drive);
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_degree = new Rotation2d(xSpeed,ySpeed).getDegrees();
  }

  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngle The turning angle in degrees
   * @param drive The drive subsystem to use
   */
  public TurnCommand(double targetAngle, DriveSubsystem drive) {
    m_drive = drive;
    addRequirements(m_drive);
    m_degree = targetAngle;
    m_xSpeed = Math.cos(Math.toRadians(targetAngle));
    m_ySpeed = Math.sin(Math.toRadians(targetAngle));
  }

  @Override
  public void initialize() {
    m_drive.setCurrentAngle(m_degree);
  }

  @Override
  public void execute() {
    m_drive.driveturning(m_xSpeed, m_ySpeed);
    SmartDashboard.putNumber("Set Angle", m_degree);
    SmartDashboard.putNumber("Swerve 1 Angle", m_drive.getModuleStates()[0].angle.getDegrees());
    SmartDashboard.putNumber("Swerve 2 Angle", m_drive.getModuleStates()[1].angle.getDegrees());
    SmartDashboard.putNumber("Swerve 3 Angle", m_drive.getModuleStates()[2].angle.getDegrees());
    SmartDashboard.putNumber("Swerve 4 Angle", m_drive.getModuleStates()[3].angle.getDegrees());
    SmartDashboard.putBoolean("Angle reached", isFinished());
    SmartDashboard.putNumber("x value", m_xSpeed);
    SmartDashboard.putNumber("y value", m_ySpeed);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return m_drive.getModulesAtAngle(m_degree);
  }
}