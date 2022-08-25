package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimelightAutotrackCommand extends CommandBase{
    private final LimelightSubsystem m_limelight;
    private final DriveSubsystem m_drive;
    private boolean m_range;
    private double  m_xSpeed;
    private double m_rot;
    private double m_TargetDistance;
    private double m_XDistance;

    public LimelightAutotrackCommand(LimelightSubsystem limelight, DriveSubsystem drive){
        m_limelight = limelight;
        addRequirements(limelight);
        m_drive = drive;
        addRequirements(drive);
        m_limelight.track();
        m_range = m_limelight.withinRange();
        m_xSpeed = m_limelight.GetXSpeed();
        m_rot = m_limelight.GetRot();
        m_TargetDistance = m_limelight.GetDistance();
        m_XDistance = m_limelight.GetXDistance();
    }

    @Override
    public void execute(){
        m_limelight.track();
        m_xSpeed = m_limelight.GetXSpeed();
        m_rot = m_limelight.GetRot();
        m_drive.drive(m_xSpeed, 0, m_rot, false);
        
    }

    @Override
    public boolean isFinished(){
        if (m_range){
            return true;
        }
        else{
            return false;
        }

    }
}