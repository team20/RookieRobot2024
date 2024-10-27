package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public record JonahsMotor(int port, boolean inverted) {
    public CANSparkMax init() {
        CANSparkMax motor = new CANSparkMax(port, MotorType.kBrushless);
        motor.setInverted(inverted);
        return motor;
    }
}
