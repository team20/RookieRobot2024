package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagSubsystem;

public class AprilTagAlignAndIDCommand extends Command {
    AprilTagSubsystem m_tagSubsystem;
    double[] m_pose;
    double m_id;

    public AprilTagAlignAndIDCommand() {
        m_tagSubsystem = AprilTagSubsystem.get();
        addRequirements(AprilTagSubsystem.get());
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_id = m_tagSubsystem.identify();
        m_pose = m_tagSubsystem.getPose();
        SmartDashboard.putNumberArray("pose", m_pose);
        SmartDashboard.putNumber("id", m_id);
    }

    @Override
    public boolean isFinished() {
        if (m_id == 1) {
            SmartDashboard.putBoolean("tagFinished", true);
            return true;
        }

        SmartDashboard.putBoolean("tagFinished", false);
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
