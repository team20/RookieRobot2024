package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import java.util.function.BooleanSupplier;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem  extends SubsystemBase {
    private CANSparkMax m_topMotor;
    private CANSparkMax m_bottomMotor;

    public ShooterSubsystem() {
        m_topMotor = new CANSparkMax(ShooterConstants.kTopPort, MotorType.kBrushless);
        m_bottomMotor = new CANSparkMax(ShooterConstants.kBottomPort, MotorType.kBrushless);
        m_topMotor.setInverted(ShooterConstants.kTopReversed);
        m_bottomMotor.setInverted(ShooterConstants.kBottomReversed);
    }

    public void bindButtons(BooleanSupplier in, BooleanSupplier out) {
        setDefaultCommand(run(() -> {
            double speedMultiplier = 0;
            if(in.getAsBoolean()) speedMultiplier -= 1;
            if(out.getAsBoolean()) speedMultiplier += 1;
            m_topMotor.set(speedMultiplier*ShooterConstants.kSpeed);
            m_bottomMotor.set(speedMultiplier*ShooterConstants.kSpeed);
        }));
    }
}
