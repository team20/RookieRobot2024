package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class CalibrationAutoCommand extends Command {
    private final DriveSubsystem m_driveSubsystem;

    public static enum Operation {
        CMD_ANGLE, CMD_DISTANCE
    }

    Operation m_op;
    double m_amount = 0; // if distance, in meters; if angle, in degrees
  /***
   * Autonomous command to steer and drive
   * 
   * @param op
   *                        Enumerated mode-steer (CMD_ANGLE) or drive (CMD_DISTANCE)
   * @param amount
   *                        op == CMD_ANGLE: amount is angle in degrees,
   *                        op == CMD_DISTANCE: amount is distance in meters
   * @see Operation
   */
    public CalibrationAutoCommand(Operation op, double amount) { 
        m_driveSubsystem = DriveSubsystem.get();
        m_op = op;
        if (m_op == Operation.CMD_DISTANCE) {
            // double currentPosition = (m_driveSubsystem.getFrontLeftSwerveModule().getDriveEncoder().getPosition());
            // SmartDashboard.putNumber("current position", currentPosition);
            m_amount = amount; 
        } else if (m_op == Operation.CMD_ANGLE) { 
            m_amount = amount;
        } else {
            try {
                throw new Exception("Unsupported command");
            } catch (Exception e) {

                e.printStackTrace();
            }
        }
        addRequirements(DriveSubsystem.get());
    }

    @Override
    public void initialize() {
        if (m_op == Operation.CMD_DISTANCE) {
            double currentPosition = (m_driveSubsystem.getFrontLeftSwerveModule().getDriveEncoder().getPosition());
            SmartDashboard.putNumber("current position", currentPosition);
            m_amount += currentPosition; 
        } else {
            SmartDashboard.putNumber("starting angle", m_driveSubsystem.getFrontLeftSwerveModule().getCANCoder().getPosition().getValueAsDouble());
        }
    }

    @Override
    public void execute() {
        // System.out.println(m_op == Operation.CMD_ANGLE);
        if (m_op == Operation.CMD_ANGLE) { 
            m_driveSubsystem.setSteerMotors(m_amount, m_amount, m_amount, m_amount);
        } else {      
            // turn on motors - set 20% power for now
            double m_powerLevel = 0.2;
            m_driveSubsystem.setDriveMotors(m_powerLevel, m_powerLevel, m_powerLevel, m_powerLevel);
        }
    }

    @Override
    public boolean isFinished() {
        switch(m_op){
            case CMD_ANGLE:
                double encAng = m_driveSubsystem.getFrontLeftSwerveModule().getCANCoder().getPosition().getValueAsDouble() % 360;
                double ang1 = encAng - 360;
                //The error between the actual angle and the target angle
                double diff1 = Math.abs(encAng - m_amount);
                double diff2 = Math.abs(ang1 - m_amount);
                boolean isDone = (Math.min(diff1, diff2) < 2); // 2 degree tolerance
                if(isDone == false)
                    return false;
                SmartDashboard.putNumber("fl_angle", m_driveSubsystem.getFrontLeftSwerveModule().getCANCoder().getPosition().getValueAsDouble());
                SmartDashboard.putNumber("bl_angle", m_driveSubsystem.getBackLeftSwerveModule().getCANCoder().getPosition().getValueAsDouble());
                SmartDashboard.putNumber("fr_angle", m_driveSubsystem.getFrontRightSwerveModule().getCANCoder().getPosition().getValueAsDouble());
                SmartDashboard.putNumber("br_angle", m_driveSubsystem.getBackRightSwerveModule().getCANCoder().getPosition().getValueAsDouble());
                return true;
            case CMD_DISTANCE:
                //Determine whether the target distance has been reached
                double currentPosition = (m_driveSubsystem.getFrontLeftSwerveModule().getDriveEncoder().getPosition());
                SmartDashboard.putNumber("currentPostion", currentPosition);
                SmartDashboard.putNumber("m_amount", m_amount);
                return (m_driveSubsystem.getFrontLeftSwerveModule().getDriveEncoder().getPosition() >= m_amount);
        }
                
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        switch(m_op){
            case CMD_ANGLE:
                SmartDashboard.putNumber("CAC end", m_amount);
                break;
            case CMD_DISTANCE:
                //Determine whether the target distance has been reached
                m_driveSubsystem.setDriveMotors(0,0,0,0);
        }
    }
}