package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import java.util.function.BooleanSupplier;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax m_motor;

    public IntakeSubsystem() {
        m_motor = new CANSparkMax(IntakeConstants.kPort, MotorType.kBrushless);
    }

    public void bindButtons(BooleanSupplier up, BooleanSupplier down) {
        setDefaultCommand(run(() -> {
            double speedMultiplier = 0;
            if(up.getAsBoolean()) speedMultiplier -= 1;
            if(down.getAsBoolean()) speedMultiplier += 1;
            SmartDashboard.putNumber("intake speed", speedMultiplier);
            m_motor.set(speedMultiplier*IntakeConstants.kSpeed);
        }));
    }
}