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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class SwerveModule {
    private PIDController m_PIDController = new PIDController(DriveConstants.kP, 0, 0);
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
        m_PIDController.setTolerance(DriveConstants.kSwerveAngleTolerance.getDegrees());
        m_driveEncoder.setPositionConversionFactor(SwerveConstants.kTicksToMeters);
    }
    /***
     * Configures our motors with the exact same settings
     * 
     * @param motorController);

    }

     *                        The CANSparkMax to configure
     */
    private static void configMotorController(CANSparkBase motorController) {
        motorController.restoreFactoryDefaults();
        motorController.setIdleMode(IdleMode.kBrake);
        motorController.enableVoltageCompensation(12);
        motorController.setSmartCurrentLimit(30);
    }

    public Rotation2d getSteerAngle() {
        return Rotation2d.fromRotations(m_CANCoder.getAbsolutePosition().getValueAsDouble());  
    }

    public Measure<Distance> getDriveEncoderPosition() {
        return Units.Meters.of(m_driveEncoder.getPosition());
    }

    public void resetDriveEncoder() {
        m_driveEncoder.setPosition(0);
    }

    public void periodic() {
        m_steerMotor.set(m_PIDController.calculate(m_CANCoder.getAbsolutePosition().getValueAsDouble()));
    }

    public void setSetpoint(Rotation2d angle) {
        m_PIDController.setSetpoint(angle.getDegrees());
    }

    public void setPower(double power) {
        m_driveMotor.set(power);
    }

    public void stopDriving() {
        m_driveMotor.stopMotor();
    }

    public boolean atSetpoint() {
        return m_PIDController.atSetpoint();
    }

    public void setModuleState(SwerveModuleState state){
        // Will allow the module to spin to 180 deg + target angle
        // but swap drive speed if that is quicker than normal
        state = SwerveModuleState.optimize(state, getSteerAngle());
        // Set drive speed
        m_driveMotor.set(state.speedMetersPerSecond * DriveConstants.kDriveScale);
        m_PIDController.setSetpoint(state.angle.getDegrees());
        //Print state to dashboard
        SmartDashboard.putString("Swerve module " + m_CANCoder.getDeviceID(), state.toString());
    }
}
