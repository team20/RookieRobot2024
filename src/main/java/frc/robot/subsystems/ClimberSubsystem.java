package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;

public class ClimberSubsystem extends SubsystemBase {
    private CANSparkMax m_leftMotor;
    private CANSparkMax m_rightMotor;

    public ClimberSubsystem() {
        m_leftMotor = new CANSparkMax(ClimberConstants.kLeftMotorPort, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(ClimberConstants.kRightMotorPort, MotorType.kBrushless);
        m_leftMotor.setInverted(ClimberConstants.kLeftReversed);
        m_rightMotor.setInverted(ClimberConstants.kRightReversed);
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