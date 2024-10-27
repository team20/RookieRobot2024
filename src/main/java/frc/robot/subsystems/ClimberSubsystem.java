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
    }

    public void bindButtons(DoubleSupplier stick) {
        setDefaultCommand(run(() -> {
            double speed = ControllerConstants.triggerDeadZone(stick.getAsDouble()) * ClimberConstants.kSpeed;
            m_rightMotor.set(speed);
            m_leftMotor.set(speed);
        }));
    }
}