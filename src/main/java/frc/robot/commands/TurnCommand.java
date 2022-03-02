package frc.robot.commands;

//mport edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ModuleConstants;


/** A command that will turn the robot to the specified angle. */
public class TurnCommand extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngle The turning angle
   * @param drive The drive subsystem to use
   */
  public TurnCommand(double targetAngle, DriveSubsystem drive) {
    super(
        new PIDController(ModuleConstants.kPModuleDriveController, ModuleConstants.kIModuleDriveController, ModuleConstants.kDModuleDriveController),
        // Close loop on heading
        drive::getHeading,
        // Set reference to target
        targetAngle,
        // Pipe output to turn robot
        output -> drive.driveturning(0, output),
        // Require the drive
        drive);

    //Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    //Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}