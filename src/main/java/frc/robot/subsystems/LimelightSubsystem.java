package frc.robot.subsystems;

import java.lang.*;
import java.lang.annotation.Target;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");
    private double tv,tx,ty;
    private double led;
    private double xSpeed = 0;
    private double rot = 0;
    private final HttpCamera limelightCameraServer = new HttpCamera("Limelight Camera", "http://10.80.15.11:5801/stream.mjpg", HttpCameraKind.kMJPGStreamer);
    private double TargetDistance = (LimelightConstants.kHeightOfTheTarget-LimelightConstants.kHeightOfTheLimelight)/Math.tan(Math.toRadians(ty+LimelightConstants.kAngleOfTheLimilight));
    private double XDistance = (Math.tan(Math.toRadians(tx))*TargetDistance);

    public LimelightSubsystem(){
        m_table.getEntry("ledMode").setNumber(0.0);
        m_table.getEntry("camMode").setNumber(1.0);
        Update();
    }

    private void Update(){
        tv = m_table.getEntry("tv").getDouble(0);
        tx = m_table.getEntry("tx").getDouble(0);
        ty = m_table.getEntry("ty").getDouble(0);
        led = m_table.getEntry("ledMode").getDouble(0);
    }
    
    public double Gettarget(){
        Update();
        return tv;
    }

    public double GetDistance(){
        Update();
        return TargetDistance;
    }

    public double GetXDistance(){
        Update();
        return XDistance;
    }

    public void OpenLight(){
        if(led == 1.0){
        m_table.getEntry("ledMode").setNumber(3.0);
        m_table.getEntry("camMode").setNumber(0.0);
        Update();
        }  
    }
    
    public void CloseLight(){
        if(led == 3.0){
        m_table.getEntry("ledMode").setNumber(1.0);
        m_table.getEntry("camMode").setNumber(0.0);
        Update();
        }
    }

    public boolean withinRange(){
        Update();
        if(TargetDistance < LimelightConstants.kDistanceWithTargetMax && TargetDistance > LimelightConstants.kDistanceWithTargetMin && tx < LimelightConstants.kAngleMax && tx > LimelightConstants.kAngleMin){
            return true;
        }
        return false;
    }

    public void track(){
        Update();

        if(tv == 1 && withinRange()){
            xSpeed = 0;
            rot = 0;
        }
        else if(tv == 1 && TargetDistance < LimelightConstants.kDistanceWithTargetMax && TargetDistance > LimelightConstants.kDistanceWithTargetMin){
            xSpeed = 0;
            rot = LimelightConstants.kLimelightFactor * tx;
            //rot = 3000;
        }
        else if(tv == 1){
            if(TargetDistance > LimelightConstants.kDistanceWithTargetMax){
                xSpeed = -0.1;
            }
            if(TargetDistance < LimelightConstants.kDistanceWithTargetMin){
                xSpeed = 0.1;
            }
            rot = 0;
        }
        else{
            xSpeed = 0;
            rot = 3000;
        }
    }

    public double GetXSpeed(){
        return xSpeed;
    }

    public double GetRot(){
        return rot;
    }
}
