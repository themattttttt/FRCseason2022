package frc.robot.testing;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import frc.robot.testing.test_constants;
import frc.robot.testing.test_constants.PIDconstant;
import frc.robot.testing.test_constants.driveConstants;

public class swModule {
    private final TalonSRX drMotor ;
    private final TalonFX turnMotor;
    private final CANCoder turnCANcoder;
    private final PIDController controllerPID = 
      new PIDController(PIDconstant.drivingKp,PIDconstant.drivingKi,PIDconstant.drivingKd);

/**
 * constructor of the swModule class. 
 * @param drMotorId
 * CAN ID of the driving motor
 * @param turnMotorId
 * CAN ID of the turning motor
 * @param turnEncoderId
 * CAN ID of the CANCoder on the swerve module
 */
    public swModule(
        int drMotorId,
        int turnMotorId,
        int turnEncoderId) {
            drMotor = new TalonSRX(drMotorId);
            turnMotor = new TalonFX(turnMotorId);
            turnCANcoder = new CANCoder(turnEncoderId);
        }

    /**
     * Speed controlling method of the swerve module
     * @param drPara
     * Driving output value, the y-axis displacement of the joystick
     * @param turnPara
     * Turning output value, angle of the 
     */    
    public void setDrive(Double drPara, Double turnPara){
        final double drivingO =
        controllerPID.calculate(drMotor.getMotorOutputPercent(), drPara);
        final double turningO = controllerPID.calculate(turnCANcoder.getPosition(),turnPara);
        drMotor.set(ControlMode.PercentOutput, drivingO);
        turnMotor.set(ControlMode.PercentOutput, turningO);
    }
    
    public void encooderSet(double initPlace){
        turnCANcoder.setPosition(initPlace);
    }
}
