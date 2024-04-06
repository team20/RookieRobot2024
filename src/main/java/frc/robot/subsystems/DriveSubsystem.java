// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private SwerveModule m_frontLeftSwerveModule;
  private SwerveModule m_frontRightSwerveModule;
  private SwerveModule m_backLeftSwerveModule;
  private SwerveModule m_backRightSwerveModule;
  private SwerveDriveKinematics m_kinematics;
  private AHRS m_gyro = new AHRS(SPI.Port.kMXP); 

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Initialize modules
    m_frontLeftSwerveModule = new SwerveModule(
      DriveConstants.kFrontLeftCANCoderPort, 
      DriveConstants.kFrontLeftDrivePort, 
      DriveConstants.kFrontLeftSteerPort, 
      DriveConstants.kFrontLeftDriveInverted
    );

    m_frontRightSwerveModule = new SwerveModule(
      DriveConstants.kFrontRightCANCoderPort, 
      DriveConstants.kFrontRightDrivePort, 
      DriveConstants.kFrontRightSteerPort, 
      DriveConstants.kFrontRightDriveInverted
    );

    m_backLeftSwerveModule = new SwerveModule(
      DriveConstants.kBackLeftCANCoderPort, 
      DriveConstants.kBackLeftDrivePort, 
      DriveConstants.kBackLeftSteerPort, 
      DriveConstants.kBackLeftDriveInverted
    );

    m_backRightSwerveModule = new SwerveModule(
      DriveConstants.kBackRightCANCoderPort, 
      DriveConstants.kBackRightDrivePort, 
      DriveConstants.kBackRightSteerPort, 
      DriveConstants.kBackRightDriveInverted
    );

    // Reset yaw to zero
    m_gyro.zeroYaw();

    Translation2d m_frontLeftLocation = new Translation2d(0.3, 0.3);
    Translation2d m_frontRightLocation = new Translation2d(0.3, -0.3);
    Translation2d m_backLeftLocation = new Translation2d(-0.3, 0.3);
    Translation2d m_backRightLocation = new Translation2d(-0.3, -0.3);

    // Creating my kinematics object using the module locations
    m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_backLeftLocation, m_frontRightLocation, m_backRightLocation
    );
  }

  /***
   * Recalculates the PID output, and uses it to drive our steer motors. Also logs
   * wheel rotations, and PID setpoints
   */
  @Override
  public void periodic() {
    // System.out.println("running"); 
    // For each of our steer motors, feed the current angle of the wheel into its
    // PID controller, and use it to calculate the duty cycle for its motor, and
    // spin the motor
    m_frontLeftSwerveModule.periodic();
    m_frontRightSwerveModule.periodic();
    m_backLeftSwerveModule.periodic();
    m_backRightSwerveModule.periodic();
  }

  public Command autoAngleCommand(Rotation2d angle) {
    return new FunctionalCommand(() -> {
      m_frontLeftSwerveModule.setSetpoint(angle);
      m_backLeftSwerveModule.setSetpoint(angle);
      m_frontRightSwerveModule.setSetpoint(angle);
      m_backLeftSwerveModule.setSetpoint(angle);
    }, () -> {}, interrupted -> {}, () -> {
      SmartDashboard.putNumber("fl_angle", m_frontLeftSwerveModule.getSteerAngle().getDegrees());
      SmartDashboard.putNumber("bl_angle", m_backLeftSwerveModule.getSteerAngle().getDegrees());
      SmartDashboard.putNumber("fr_angle", m_frontRightSwerveModule.getSteerAngle().getDegrees());
      SmartDashboard.putNumber("br_angle", m_backRightSwerveModule.getSteerAngle().getDegrees());

      return m_frontLeftSwerveModule.atSetpoint() &&
             m_backLeftSwerveModule.atSetpoint() &&
             m_frontRightSwerveModule.atSetpoint() &&
             m_backRightSwerveModule.atSetpoint();
    }, this);
  }

  public Command autoDistanceCommand(Measure<Distance> distance) {
    return new FunctionalCommand(() -> {
      m_frontLeftSwerveModule.resetDriveEncoder();
      double power = 0.2 * DriveConstants.kDriveScale;
      m_frontLeftSwerveModule.setPower(power);
      m_backLeftSwerveModule.setPower(power);
      m_frontRightSwerveModule.setPower(power);
      m_backRightSwerveModule.setPower(power);
    }, () -> {}, interrupted -> {
      m_frontLeftSwerveModule.stopDriving();
      m_backLeftSwerveModule.stopDriving();
      m_frontRightSwerveModule.stopDriving();
      m_backRightSwerveModule.stopDriving();
    }, () -> {
      return m_frontLeftSwerveModule.getDriveEncoderPosition().gt(distance);
    }, this);
  }

  public void setDefaultDriveCommand(DoubleSupplier xAxisDrive, DoubleSupplier yAxisDrive,
      DoubleSupplier rotationAxis) {
    setDefaultCommand(run(() -> {
      // UPDATED to use wpilib swerve calculations
      // Get the foward, strafe, and rotation speed, using a deadband on the joystick
      // input so slight movements don't move the robot
      double fwdSpeed = MathUtil.applyDeadband(yAxisDrive.getAsDouble(), ControllerConstants.kDeadzone);
      double strSpeed = MathUtil.applyDeadband(xAxisDrive.getAsDouble(), ControllerConstants.kDeadzone);
      double rotSpeed = MathUtil.applyDeadband(rotationAxis.getAsDouble(), ControllerConstants.kDeadzone);

      ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        fwdSpeed, strSpeed, rotSpeed, Rotation2d.fromDegrees(-m_gyro.getYaw()));

      // Now use this in our kinematics
      SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.kMaxVelocity);

      m_frontLeftSwerveModule.setModuleState(moduleStates[0]);
      m_backLeftSwerveModule.setModuleState(moduleStates[1]);
      m_frontRightSwerveModule.setModuleState(moduleStates[2]);
      m_backRightSwerveModule.setModuleState(moduleStates[3]);
      
      // SmartDashboard logging
      SmartDashboard.putNumber("Foward Speed", fwdSpeed);
      SmartDashboard.putNumber("Strafe Speed", strSpeed);
      SmartDashboard.putNumber("Rotation Speed", rotSpeed);
    }));
  }
}
