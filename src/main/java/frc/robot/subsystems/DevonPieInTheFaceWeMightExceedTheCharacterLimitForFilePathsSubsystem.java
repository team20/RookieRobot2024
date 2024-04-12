package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import java.util.function.BooleanSupplier;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIFConstants;

public class DevonPieInTheFaceWeMightExceedTheCharacterLimitForFilePathsSubsystem  extends SubsystemBase {
    private CANSparkMax m_kickMotor;
    private CANSparkMax m_shootMotor;
    private CANSparkMax m_passMotor;
    private CANSparkMax m_intakeMotor;

    public DevonPieInTheFaceWeMightExceedTheCharacterLimitForFilePathsSubsystem() {
        // Get ready to practice your soccer drills! Kick, shoot, pass!
        m_kickMotor = new CANSparkMax(PIFConstants.kKickPort, MotorType.kBrushless);
        m_shootMotor = new CANSparkMax(PIFConstants.kShootPort, MotorType.kBrushless);
        m_passMotor = new CANSparkMax(PIFConstants.kPassPort, MotorType.kBrushless);
        m_intakeMotor = new CANSparkMax(PIFConstants.kIntakePort, MotorType.kBrushless);
    }

    public void setDefaultPIFCommand(BooleanSupplier up, BooleanSupplier down, BooleanSupplier kickOut, BooleanSupplier kickIn, BooleanSupplier shootOut, BooleanSupplier shootIn) {
        setDefaultCommand(run(() -> {
            double speedMultiplier = 0;
            if(up.getAsBoolean()) speedMultiplier += 1;
            if(down.getAsBoolean()) speedMultiplier -= 1;
            m_intakeMotor.set(speedMultiplier*PIFConstants.kIntakeSpeed);
            m_passMotor.set(speedMultiplier*PIFConstants.kPassSpeed);

            double kickSpeed = 0;
            if(kickOut.getAsBoolean()) kickSpeed += PIFConstants.kKickSpeed;
            if(kickIn.getAsBoolean()) kickSpeed -= PIFConstants.kKickSpeed;
            m_kickMotor.set(kickSpeed);

            double shootSpeed = 0;
            if(shootOut.getAsBoolean()) shootSpeed += PIFConstants.kShootSpeed;
            if(shootIn.getAsBoolean()) shootSpeed -= PIFConstants.kShootSpeed;
            m_shootMotor.set(shootSpeed);
        }));
    }
}
