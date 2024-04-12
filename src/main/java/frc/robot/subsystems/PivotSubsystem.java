package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
    private CANSparkMax m_motor;

    public PivotSubsystem() {
        m_motor = new CANSparkMax(PivotConstants.kMotorPort, MotorType.kBrushless);
        m_motor.setIdleMode(IdleMode.kBrake);
    }

    public void setDefaultPivotCommand(BooleanSupplier left, BooleanSupplier right) {
        setDefaultCommand(run(() -> {
            double speed = 0;
            if(left.getAsBoolean()) speed += PivotConstants.kSpeed;
            if(right.getAsBoolean()) speed -= PivotConstants.kSpeed;
            m_motor.set(speed);
        }));
    }
}
