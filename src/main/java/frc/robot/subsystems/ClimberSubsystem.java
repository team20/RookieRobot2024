package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;

public class ClimberSubsystem extends SubsystemBase {
    private CANSparkMax m_leftMotor;
    private CANSparkMax m_rightMotor;

    public ClimberSubsystem() {
        m_leftMotor = new CANSparkMax(ClimberConstants.kLeftMotorPort, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(ClimberConstants.kRightMotorPort, MotorType.kBrushless);
        
        m_leftMotor.follow(m_rightMotor);
    }

    public void bindButtons(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
        setDefaultCommand(run(() -> {
            m_rightMotor.set(MathUtil.applyDeadband(((rightTrigger.getAsDouble() + 1) / 2), ControllerConstants.kTriggerDeadzone) * ClimberConstants.kSpeed);
            m_rightMotor.set(MathUtil.applyDeadband(-((leftTrigger.getAsDouble() + 1) / 2), ControllerConstants.kTriggerDeadzone) * ClimberConstants.kSpeed);
        }));
    }
}