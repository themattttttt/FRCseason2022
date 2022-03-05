package frc.robot.commands;



import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.ArmConstants;


/** A command that will turn the robot to the specified angle. */
public class ArmCommand extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetSpeed The speed of motor
   * @param drive The drive subsystem to use
   */
  public ArmCommand( double Output,TrapezoidProfile.State setpoint,ArmSubsystem arm) {
    super(
        new PIDController(ArmConstants.kPArmController, ArmConstants.kIArmController, ArmConstants.kDArmController),
        // Close loop on heading
        arm::getHeading,
        // Set reference to target
        0,
        // Pipe output to turn robot
        output -> arm.useOutput(Output,setpoint),
        // Require the drive
        arm);

    // Set the controller to be continuous (because it is an angle controller)
    //getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    //getController()
      //  .setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}