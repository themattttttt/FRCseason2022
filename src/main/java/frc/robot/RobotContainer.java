// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.lang.Object;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DefenseXCommand;
import frc.robot.commands.DefenseOCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import java.util.List;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  public final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final JoystickButton XButton=new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton OButton=new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
  private final JoystickButton rightButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  private final JoystickButton leftButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  private final JoystickButton forwardButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  private final JoystickButton backwardButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  private final JoystickButton startButton = new JoystickButton(m_driverController, XboxController.Button.kStart.value);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    
    m_robotDrive.setDefaultCommand(
        // Run parallel moving, then stop at the end.
        new RunCommand(()->m_robotDrive.drive(-m_driverController.getLeftY(),m_driverController.getLeftX(),0.0,false)
            , m_robotDrive)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
      XButton.whenHeld(new DefenseXCommand(m_robotDrive));
      OButton.whenHeld(new DefenseOCommand(m_robotDrive));
      rightButton.whenHeld(new TurnCommand(0,-1,m_robotDrive).andThen(new DriveCommand(1.0, m_robotDrive)));
      leftButton.whenHeld(new TurnCommand(0,1,m_robotDrive).andThen(new DriveCommand(1.0, m_robotDrive)));
      forwardButton.whenHeld(new TurnCommand(0,0,m_robotDrive).andThen(new DriveCommand(1.0, m_robotDrive)));
      backwardButton.whenHeld(new TurnCommand(-1,0,m_robotDrive).andThen(new DriveCommand(1.0, m_robotDrive)));
      startButton.whenHeld(new TurnCommand(0, m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
