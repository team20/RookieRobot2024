package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import java.util.function.BooleanSupplier;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem  extends SubsystemBase {
    private CANSparkMax m_topMotor;
    private CANSparkMax m_bottomMotor;
    private CANSparkMax m_kickMotor;

    public ShooterSubsystem() {
        m_topMotor = new CANSparkMax(ShooterConstants.kTopPort, MotorType.kBrushless);
        m_bottomMotor = new CANSparkMax(ShooterConstants.kBottomPort, MotorType.kBrushless);
        m_kickMotor = new CANSparkMax(ShooterConstants.kKickPort, MotorType.kBrushless);
        m_topMotor.setInverted(ShooterConstants.kTopReversed);
        m_bottomMotor.setInverted(ShooterConstants.kBottomReversed);
        m_kickMotor.setInverted(ShooterConstants.kKickReversed);
        m_bottomMotor.follow(m_topMotor);
    }

    public void bindButtons(BooleanSupplier shootIn, BooleanSupplier shootOut, BooleanSupplier kickIn, BooleanSupplier kickOut) {
        setDefaultCommand(run(() -> {
            boolean in = shootIn.getAsBoolean();
            boolean out = shootOut.getAsBoolean();
            m_topMotor.set((in && !out ? 1 : out && !in ? -1 : 0) * ShooterConstants.kShootSpeed);
            in = kickIn.getAsBoolean();
            out = kickOut.getAsBoolean();
            m_kickMotor.set((in && !out ? 1 : out && !in ? -1 : 0) * ShooterConstants.kKickSpeed);
        }));
    }
}
