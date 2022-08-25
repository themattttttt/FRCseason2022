// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.lang.Object;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperateConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DefenseXCommand;
import frc.robot.commands.ArmHoldCommand;
import frc.robot.commands.ArmBackwardTempCommand;
import frc.robot.commands.ArmForwardTempCommand;
import frc.robot.commands.DefenseOCommand;
import frc.robot.commands.ElevatorDownCommand;
import frc.robot.commands.ElevatorUpCommand;
import frc.robot.commands.IntakeNegativeCommand;
import frc.robot.commands.IntakePositiveCommand;
import frc.robot.commands.ShooterUpperCommand;
import frc.robot.commands.ShooterLowerCommand;
import frc.robot.commands.LimelightAutotrackCommand;
import frc.robot.commands.LimelightChangeLightCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterUpperSubsystem;
import frc.robot.subsystems.ShooterLowerSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Pneumatic;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.commands.PneumaticRetractCommand;
import frc.robot.commands.PneumaticReleaseCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterUpperSubsystem m_uppershooter = new ShooterUpperSubsystem(OperateConstants.kUpperShooterChannel);
  private final ShooterLowerSubsystem m_lowershooter = new ShooterLowerSubsystem(OperateConstants.kLowerShooterChannel);
  private final Pneumatic m_pnematic = new Pneumatic(0);
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final LimelightSubsystem m_limelight = new LimelightSubsystem();


  // The driver's controller
  public final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public final XboxController m_operateController = new XboxController(OIConstants.kOperateControllerPort);

  //Create buttons
  private final JoystickButton XButton = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton OButton = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
  /*
  private final JoystickButton rightButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  private final JoystickButton leftButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  private final JoystickButton forwardButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  private final JoystickButton backwardButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  */
  private final JoystickButton startButton = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
  private final JoystickButton menuButton = new JoystickButton(m_driverController, XboxController.Button.kBack.value);

  private final JoystickButton ShootButton = new JoystickButton(m_operateController, XboxController.Button.kB.value);
  private final JoystickButton ShootOutButton = new JoystickButton(m_operateController, XboxController.Button.kA.value);
  private final JoystickButton ReleaseButton = new JoystickButton(m_operateController, XboxController.Button.kY.value);
  private final JoystickButton RetractButton = new JoystickButton(m_operateController, XboxController.Button.kX.value);
  private final JoystickButton LightButton = new JoystickButton(m_operateController, XboxController.Button.kStart.value);
  private final JoystickButton holdButton = new JoystickButton(m_operateController, XboxController.Button.kBack.value);

  private final JoystickButton armForwardButton = new JoystickButton(m_operateController,XboxController.Button.kRightBumper.value);
  private final JoystickButton armBackwardButton = new JoystickButton(m_operateController,XboxController.Button.kLeftBumper.value);


  //private final JoystickButton driverIntakePositiveButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  //private final JoystickButton driverIntakeNegativeButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  private final Trigger elevatorUpTrigger = new Trigger(()->m_operateController.getLeftY()<-0.1);
  private final Trigger elevatorDownTrigger = new Trigger(()->m_operateController.getLeftY()>0.1);
  private final Trigger intakePositiveTrigger = new Trigger(()->(m_operateController.getPOV()==0 || m_driverController.getXButton()));
  private final Trigger intakeNegativeTrigger = new Trigger(()->(m_operateController.getPOV()==180 || m_driverController.getAButton()));
  private final Trigger TrackTrigger = new Trigger(()-> m_operateController.getLeftTriggerAxis()> 0.1);
  private final Trigger holdTrigger = new Trigger(()->!(m_operateController.getLeftBumper() || m_operateController.getRightBumper()));

  
  private final Trigger forwardTrigger = new Trigger(()->m_driverController.getPOV() == 0);
  private final Trigger backwardTrigger = new Trigger(()->m_driverController.getPOV() == 180);
  private final Trigger leftTrigger = new Trigger(()->m_driverController.getPOV() == 270);
  private final Trigger rightTrigger = new Trigger(()->m_driverController.getPOV() == 90);
 


 //private final JoystickButton intakeInButton = new JoystickButton(m_operateController,XboxController.)

  public final Command resetCommand = new RunCommand(()->m_robotDrive.setAllToZero());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // Run parallel moving, then stop at the end.
        new RunCommand(()->m_robotDrive.drive(-JoystickCurver(m_driverController.getLeftY()),JoystickCurver(m_driverController.getLeftX()),DriveConstants.kAngularSpeed*(-m_driverController.getRightTriggerAxis()+ m_driverController.getLeftTriggerAxis()),false)
            , m_robotDrive)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */

  private double JoystickCurver(double joystickread){
      if(joystickread > 0.03){ //magic number is the thresold of the joystick getleftY or getleftX
        return Math.pow(joystickread, 2);
      }
      if (joystickread < -0.03){
        return - Math.pow(joystickread, 2);
      }
      return 0.0;
  }

  private void configureButtonBindings() {
      XButton.whenHeld(new DefenseXCommand(m_robotDrive));
      OButton.whenHeld(new DefenseOCommand(m_robotDrive));
      rightTrigger.whileActiveContinuous(new RunCommand(()->m_robotDrive.drive(0, -1, 0, false), m_robotDrive));
      leftTrigger.whileActiveContinuous(new RunCommand(()->m_robotDrive.drive(0, 1, 0, false), m_robotDrive));
      forwardTrigger.whileActiveContinuous(new RunCommand(()->m_robotDrive.drive(1, 0, 0, false), m_robotDrive));
      backwardTrigger.whileActiveContinuous(new RunCommand(()->m_robotDrive.drive(-1, 0, 0, false), m_robotDrive));
      startButton.whenHeld(resetCommand);
      ReleaseButton.whenHeld(new PneumaticReleaseCommand(m_pnematic));
      RetractButton.whenHeld(new PneumaticRetractCommand(m_pnematic));
      holdTrigger.whileActiveContinuous(new ArmHoldCommand(m_arm));

      ShootButton.toggleWhenPressed(new ShooterUpperCommand(m_uppershooter));
      ShootOutButton.whenHeld(new ShooterLowerCommand(m_lowershooter));
      armForwardButton.whenHeld(new ArmForwardTempCommand(m_arm));
      armBackwardButton.whenHeld(new ArmBackwardTempCommand(m_arm));
      LightButton.whenPressed(new LimelightChangeLightCommand(m_limelight));

      elevatorUpTrigger.whileActiveContinuous(new ElevatorUpCommand(m_elevator));
      elevatorDownTrigger.whileActiveContinuous(new ElevatorDownCommand(m_elevator));
      
      intakePositiveTrigger.whileActiveContinuous(new IntakePositiveCommand(m_intake));
      intakeNegativeTrigger.whileActiveContinuous(new IntakeNegativeCommand(m_intake));

      TrackTrigger.whileActiveContinuous(new LimelightAutotrackCommand(m_limelight, m_robotDrive));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    final Command AutoDriveCommand = new RunCommand(()->m_robotDrive.drive(0.5, 0, 0, false)).withTimeout(1.0);
    final Command AutoTrack = new LimelightAutotrackCommand(m_limelight,m_robotDrive);
    final Command AutoShoot = new ShooterUpperCommand(m_uppershooter);
    return AutoDriveCommand.andThen(AutoTrack).andThen(AutoShoot);
  }

  public Command getTestCommand(){
    final Command Wait = new WaitCommand(3.0);
    final Command TestDriveOCommand = new DefenseOCommand(m_robotDrive).withTimeout(3.0);
    final Command TestArmForwardTempCommand = new ArmForwardTempCommand(m_arm).withTimeout(1.0);
    final Command TestArmBackwadTempCmmand = new ArmBackwardTempCommand(m_arm).withTimeout(1.0);
    final Command TestElevatorUpCommand = new ElevatorUpCommand(m_elevator).withTimeout(1.0);
    final Command TestElevatorDownCommand = new ElevatorDownCommand(m_elevator).withTimeout(1.0);
    final Command TestPneumaticReleaseCommand = new PneumaticReleaseCommand(m_pnematic);
    final Command TestPneumaticRetractCommand = new PneumaticRetractCommand(m_pnematic);
    final Command TestIntakePositiveCommand = new IntakePositiveCommand(m_intake).withTimeout(1.0);
    final Command TestIntakeNegativeCommand = new IntakeNegativeCommand(m_intake).withTimeout(1.0);
    final Command TestShoooooooooter = new ShooterUpperCommand(m_uppershooter).withTimeout(1.0);
    
    final Command TestCommand = TestDriveOCommand.andThen(Wait).
                                                  andThen(TestArmForwardTempCommand).andThen(Wait).
                                                  andThen(TestArmBackwadTempCmmand).andThen(Wait).
                                                  andThen(TestElevatorUpCommand).andThen(Wait).
                                                  andThen(TestElevatorDownCommand).andThen(Wait).
                                                  andThen(TestPneumaticReleaseCommand).andThen(Wait).
                                                  andThen(TestIntakePositiveCommand).andThen(Wait).
                                                  andThen(TestIntakeNegativeCommand).andThen(Wait).
                                                  andThen(TestPneumaticRetractCommand).andThen(Wait).
                                                  andThen(TestShoooooooooter);

    return TestCommand;
  }

}
