package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;

public class ClimberSubsystem extends SubsystemBase {
    private CANSparkMax m_leftMotor;
    private CANSparkMax m_rightMotor;

    public ClimberSubsystem() {
        m_leftMotor = ClimberConstants.kLeftMotor.init();
        m_rightMotor = ClimberConstants.kRightMotor.init();
        m_leftMotor.follow(m_rightMotor);
    }

    public void bindButtons(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
        setDefaultCommand(run(() -> {
            double speed = ControllerConstants.triggerDeadZone((rightTrigger.getAsDouble() + 1) / 2);
            speed -= ControllerConstants.triggerDeadZone((leftTrigger.getAsDouble() + 1) / 2);
            m_rightMotor.set(speed * ClimberConstants.kSpeed);
        }));
    }
}