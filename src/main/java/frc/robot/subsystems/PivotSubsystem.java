package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
    private CANSparkMax m_motor;

    public PivotSubsystem() {
        m_motor = PivotConstants.kMotor.init();
        m_motor.setIdleMode(IdleMode.kBrake);
    }

    public void bindButtons(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
        setDefaultCommand(run(() -> {
            double speed = ControllerConstants.triggerDeadZone(rightTrigger.getAsDouble() + 1);
            speed -= ControllerConstants.triggerDeadZone(leftTrigger.getAsDouble() + 1);
            m_motor.set(speed * PivotConstants.kSpeed);
        }));
    }
}