package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;


/** A robot arm subsystem that moves with a motion profile. */
public class ArmSubsystem extends ProfiledPIDSubsystem {
  //很简单吧新建一个电机，kArmMotorport 要改成电机的can ，现在还没设置
  private final TalonFX m_motor = new TalonFX(ArmConstants.kArmMotorPort);
  private final TalonFX m_motor2 = new TalonFX(ArmConstants.kArmMotorPort2);

  /** Create a new ArmSubsystem. */
  public ArmSubsystem() { //这玩意叫constructor，就是初始化一个 ArmSubsystem 的物体，里面的代码只跑一遍，包含一些初始设置
    super(
        new ProfiledPIDController(            
            ArmConstants.kPArmController,         ///这几个是PID参数，要调
            0,
            0,
            new TrapezoidProfile.Constraints(     ///设定速度和加速度最大值
                ArmConstants.kMaxVelocityRadPerSecond,
                ArmConstants.kMaxAccelerationRadPerSecSquared)),
        0);
    // Start arm at rest in neutral position  直觉告诉我这个是初始位置，参数要调（电机一开始的角度，单位rad） 
    setGoal(ArmConstants.kArmOffsetRads);
    
    //For motor, use Falcon integrated sensor as PID controller
    //set motor profiles
    TalonFXConfiguration talon_configs = new TalonFXConfiguration(); 
		/* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
		talon_configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    talon_configs.diff0Term = FeedbackDevice.IntegratedSensor;
    talon_configs.sum0Term = FeedbackDevice.IntegratedSensor;
    //以上的四行代码都是在初始化（我觉得也可以理解为配对）Falcon 的 Integrated sensor （读电机转了几个单位）

    m_motor.setNeutralMode(NeutralMode.Brake); 
    m_motor2.setNeutralMode(NeutralMode.Brake);//这行代码意思是电机通电时锁住
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) { //setpoint是目标值
    // Calculate the feedforward from the setpoint
    int kMeasuredPosHorizontal = 840; //Position measured when arm is horizontal 机械臂水平时电机的位置
    double kTicksPerDegree = 4096 / 360; //Sensor is 1:1 with arm rotation 这个数字要改，因为不是电机转一圈机械臂就转一圈，咱们有降速齿轮，要乘个系数，你来算
    double currentPos = m_motor.getSelectedSensorPosition(); //这个能够读到目前电机的位置
    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree; //计算机械臂的角度
    double radians = java.lang.Math.toRadians(degrees); //把上面的换成radian单位
    double cosineScalar = java.lang.Math.cos(radians); //算个cosine
    
    // Add the feedforward to the PID output to get the motor output
    double maxGravityFF = 0.07;  //这是理想状态下，机械臂水平时克服重力维持稳定所需要的output
    m_motor.set(ControlMode.MotionMagic, setpoint.position, DemandType.ArbitraryFeedForward, maxGravityFF * cosineScalar);
    m_motor2.follow(m_motor);
    //                这是模式（设定目标）      目标值                  额外需要的模式                 需要在output上加的值
    
  }

  @Override
  public double getMeasurement() {
    return m_motor.getSelectedSensorPosition() + ArmConstants.kArmOffsetRads;
  }
  public double getHeading() {
    return 0;
  }
}