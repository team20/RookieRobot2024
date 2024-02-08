package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ModuleConstants;

public class ShooterSubsystem {
    ShooterSubsystem s_shooter;
    CANSparkMax m_transportLeftMotor;
    CANSparkMax m_transportRightMotor;
    CANSparkMax m_shootMotor;
    CANSparkMax m_flipMotor;

    public ShooterSubsystem() {
        //Initialize variables
        s_shooter = this;
        m_shootMotor = new CANSparkMax(ModuleConstants.kShooterCANCoderPort, MotorType.kBrushless);
        m_flipMotor = new CANSparkMax(ModuleConstants.kFlipCANCoderPort, MotorType.kBrushless);
        m_transportLeftMotor = new CANSparkMax(ModuleConstants.kTransportLeftCANCoder, MotorType.kBrushless);
        m_transportRightMotor = new CANSparkMax(ModuleConstants.kTransportRightCANCoder, MotorType.kBrushless);
    }

    public void Shoot() {
        m_transportLeftMotor.set(0.5);
        m_transportRightMotor.set(0.5);
        m_shootMotor.set(0.5);

        Timer.delay(2);

        m_transportLeftMotor.set(0);
        m_transportRightMotor.set(0);
        m_shootMotor.set(0);
    }
}
