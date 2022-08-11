package frc.robot.subsystems;

import java.lang.*;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");
    private double tv,tx,ty;
    private double led;
    private double xSpeed = 0;
    private double rot = 0;
    private double TargetDistance = (LimelightConstants.kHeightOfTheTarget-LimelightConstants.kHeightOfTheLimelight)/Math.tan(Math.toRadians(ty+LimelightConstants.kAngleOfTheLimilight));
    private double XDistance = (Math.tan(Math.toRadians(tx))*TargetDistance)-14;

    public LimelightSubsystem(){
        m_table.getEntry("ledMode").setNumber(1.0);
        Update();
    }

    private void Update(){
        tv = m_table.getEntry("tv").getDouble(0.0);
        tx = m_table.getEntry("tx").getDouble(0.0);
        ty = m_table.getEntry("ty").getDouble(0.0);
        led = m_table.getEntry("ledMode").getDouble(3.0);
    }
    
    public double Gettarget(){
        Update();
        return tv;
    }

    public double GetX(){
        Update();
        return tx;
    }

    public double GetY(){
        Update();
        return ty;
    }

    public void ChangeLight(){
        Update();
        if (led == 3.0){
            m_table.getEntry("ledMode").setNumber(1.0);
        }
        else{
            m_table.getEntry("ledMode").setNumber(3.0);
        }
    }

    public void track(){
        if(tv == 1 & TargetDistance > LimelightConstants.kDistanceWithTargetMax){
            xSpeed = 3000;
            rot = 0;
        }
        else if(tv == 1 & TargetDistance < LimelightConstants.kDistanceWithTargetMin){
            xSpeed = -3000;
            rot = 0;
        }
        else if(tv == 1 & XDistance > LimelightConstants.kDistanceXMax){
            xSpeed = 0;
            rot = Math.atan(XDistance / TargetDistance) / LimelightConstants.kLimelighttime;
        }
        else if(tv == 1 & XDistance < LimelightConstants.kDistanceXMin){
            xSpeed = 0;
            rot = -Math.atan(XDistance / TargetDistance) / LimelightConstants.kLimelighttime;
        }
        else if(tv == 0){
            xSpeed = 0;
            rot = 2000;
        }
        else {
            xSpeed = 0;
            rot = 0;
        }
    }

    public double GetXSpeed(){
        return xSpeed;
    }

    public double GetRot(){
        return rot;
    }
}
