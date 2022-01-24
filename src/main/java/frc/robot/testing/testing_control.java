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
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.testing.swModule;
import frc.robot.testing.test_constants;
import frc.robot.testing.test_constants.CANCoder;
import frc.robot.testing.test_constants.Motor;

public class testing_control {
    private final swModule left_frontModule = new swModule(Motor.left_frontFalcon, Motor.left_front775, CANCoder.left_frontCoder);
    private final swModule right_frontModule = new swModule(Motor.right_frontFalcon, Motor.right_front775, CANCoder.right_frontCoder);
    
    public void joystickControl(Double speed, Double angle){
        left_frontModule.setDrive(speed, angle);
        right_frontModule.setDrive(speed, angle);
    }

}