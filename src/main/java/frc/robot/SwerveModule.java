// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class SwerveModule {
    private PIDController m_PIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD, DriveConstants.kSteerPeriod);
    private CANcoder m_CANCoder;
    private CANSparkFlex m_driveMotor;
    public RelativeEncoder m_driveEncoder;
    private CANSparkMax m_steerMotor;

    public SwerveModule(int CANport, int drivePort, int steerPort, boolean inverted){
        m_CANCoder = new CANcoder(CANport);
        m_driveMotor = new CANSparkFlex(drivePort, MotorType.kBrushless);
        m_steerMotor = new CANSparkMax(steerPort, MotorType.kBrushless);
        m_driveEncoder = m_driveMotor.getEncoder();
        configMotorController(m_driveMotor);
        m_driveMotor.setInverted(inverted);
        configMotorController(m_steerMotor);
        m_PIDController.enableContinuousInput(0, 360);
        m_driveEncoder.setPositionConversionFactor(SwerveConstants.kTicksToMeters);
    }
    /***
     * Configures our motors with the exact same settings
     * 
     * @param motorController);

    }

     *                        The CANSparkMax to configure
     */
    public static void configMotorController(CANSparkBase motorController) {
        motorController.restoreFactoryDefaults();
        motorController.setIdleMode(IdleMode.kBrake);
        motorController.enableVoltageCompensation(12);
        motorController.setSmartCurrentLimit(30);
    }

    public void updatePID() {
        m_steerMotor.set(m_PIDController.calculate(getPosition().angle.getDegrees()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_driveEncoder.getPosition(),
            Rotation2d.fromRotations(m_CANCoder.getAbsolutePosition().getValueAsDouble())
        );
    }

    public void setModuleState(SwerveModuleState state){
        // Will allow the module to spin to 180 deg + target angle
        // but swap drive speed if that is quicker than normal
        state = SwerveModuleState.optimize(state, getPosition().angle);
        // Set drive speed
        m_driveMotor.set(state.speedMetersPerSecond * DriveConstants.kDriveScale);
        m_PIDController.setSetpoint(state.angle.getDegrees());
        //Print state to dashboard
        SmartDashboard.putString("Swerve module " + m_CANCoder.getDeviceID(), state.toString());
    }
}
