package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase {
    NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tid;
    NetworkTableEntry botpose;

    static AprilTagSubsystem s_subsystem = new AprilTagSubsystem();

    AprilTagSubsystem() {
        tid = m_table.getEntry("tid");
        botpose = m_table.getEntry("botpose_targetspace");
        s_subsystem = this;
    }

    public double[] getPose() {
        return botpose.getDoubleArray(new double[6]);
    }

    public double identify() {
        return tid.getDouble(0.0);
    }

    public static AprilTagSubsystem get() {
        return s_subsystem;
    }
}
