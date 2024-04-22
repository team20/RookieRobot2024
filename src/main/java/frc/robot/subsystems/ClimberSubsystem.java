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
    }

    public void bindButtons(DoubleSupplier leftMotor, DoubleSupplier rightMotor) {
        setDefaultCommand(run(() -> {
            m_leftMotor.set(MathUtil.applyDeadband(leftMotor.getAsDouble(), ControllerConstants.kDeadzone));
            m_rightMotor.set(MathUtil.applyDeadband(rightMotor.getAsDouble(), ControllerConstants.kDeadzone));
        }));
    }
}