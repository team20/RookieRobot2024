// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

public class DriveSubsystem extends SubsystemBase {
  private SwerveModule m_frontLeftSwerveModule;
  private SwerveModule m_frontRightSwerveModule;
  private SwerveModule m_backLeftSwerveModule;
  private SwerveModule m_backRightSwerveModule;
  private static DriveSubsystem s_subsystem;
  private AHRS m_gyro = new AHRS(SPI.Port.kMXP); 
  private MedianFilter filter = new MedianFilter(5);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Singleton
    if (s_subsystem != null) {
      try {
        throw new Exception("Motor subsystem already initalized!");
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
    s_subsystem = this;

    // Initialize modules
    {
      m_frontLeftSwerveModule = new SwerveModule(
        DriveConstants.kFrontLeftCANCoderPort, 
        DriveConstants.kFrontLeftDrivePort, 
        DriveConstants.kFrontLeftSteerPort, 
        SwerveConstants.FrontLeftZero, 
        DriveConstants.kFrontLeftDriveInverted
        );

      m_frontRightSwerveModule = new SwerveModule(
        DriveConstants.kFrontRightCANCoderPort, 
        DriveConstants.kFrontRightDrivePort, 
        DriveConstants.kFrontRightSteerPort, 
        SwerveConstants.FrontRightZero, 
        DriveConstants.kFrontRightDriveInverted);

      m_backLeftSwerveModule = new SwerveModule(
        DriveConstants.kBackLeftCANCoderPort, 
        DriveConstants.kBackLeftDrivePort, 
        DriveConstants.kBackLeftSteerPort, 
        SwerveConstants.BackLeftZero, 
        DriveConstants.kBackLeftDriveInverted);

      m_backRightSwerveModule = new SwerveModule(
        DriveConstants.kBackRightCANCoderPort, 
        DriveConstants.kBackRightDrivePort, 
        DriveConstants.kBackRightSteerPort, 
        SwerveConstants.BackRightZero, 
        DriveConstants.kBackRightDriveInverted);
    }
  }
  double oldVal;
  public double getHeading(){
    return -m_gyro.getYaw();
    //return filter.calculate(-m_gyro.getYaw());
  }

  public AHRS getNavx(){
    return m_gyro;
  }

  public void resetHeading(){
    m_gyro.reset();
  }

  public static DriveSubsystem get() {
    return s_subsystem;
  }

  public void resetEncoders() {
    // Zero drive encoders
    m_frontLeftSwerveModule.getDriveEncoder().setPosition(0);
    m_frontRightSwerveModule.getDriveEncoder().setPosition(0);
    m_backLeftSwerveModule.getDriveEncoder().setPosition(0);
    m_backRightSwerveModule.getDriveEncoder().setPosition(0);
  }

  public void setWheelRotationToZeroDegrees() {
    setSteerMotors(0, 0, 0, 0);
  }

  /**
   * Makes our drive motors spin at the specified speeds
   * 
   * @param frontLeftSpeed
   *                        Speed of the front left wheel in duty cycles [-1, 1]
   * @param frontRightSpeed
   *                        Speed of the front right wheel in duty cycles [-1, 1]
   * @param backLeftSpeed
   *                        Speed of the back left wheel in duty cycles [-1, 1]
   * @param backRightSpeed
   *                        Speed of the back right wheel in duty cycles [-1, 1]
   */
  public void setDriveMotors(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed,
      double backRightSpeed) {
    m_frontLeftSwerveModule.getDriveMotor().set(frontLeftSpeed * DriveConstants.kDriveScale);
    m_frontRightSwerveModule.getDriveMotor().set(frontRightSpeed * DriveConstants.kDriveScale);
    m_backLeftSwerveModule.getDriveMotor().set(backLeftSpeed * DriveConstants.kDriveScale);
    m_backRightSwerveModule.getDriveMotor().set(backRightSpeed * DriveConstants.kDriveScale);
  }

  /***
   * Sets the target angles in degrees for each wheel on the robot
   * 
   * @param frontLeftAngle
   *                        The target angle of the front left wheel in degrees
   * @param frontRightAngle
   *                        The target angle of the front right wheel in degrees
   * @param backLeftAngle
   *                        The target angle of the back left wheel in degrees
   * @param backRightAngle
   *                        The target angle of the back right wheel in degrees
   */
  public void setSteerMotors(double frontLeftAngle, double frontRightAngle, double backLeftAngle,
      double backRightAngle) {
    m_frontLeftSwerveModule.getPIDController().setSetpoint(frontLeftAngle);
    m_frontRightSwerveModule.getPIDController().setSetpoint(frontRightAngle);
    m_backLeftSwerveModule.getPIDController().setSetpoint(backLeftAngle);
    m_backRightSwerveModule.getPIDController().setSetpoint(backRightAngle);
  }

  public void setSwerveStates(SwerveModuleState[] moduleStates){

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.kMaxVelocity);
    // Front left module state
    SwerveModuleState frontLeft = moduleStates[0];

    // Front right module state
    SwerveModuleState frontRight = moduleStates[1];

    // Back left module state
    SwerveModuleState backLeft = moduleStates[2];

    // Back right module state
    SwerveModuleState backRight = moduleStates[3];

    m_frontLeftSwerveModule.setModuleState(frontLeft);
    m_frontRightSwerveModule.setModuleState(frontRight);
    m_backLeftSwerveModule.setModuleState(backLeft);
    m_backRightSwerveModule.setModuleState(backRight);
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

    m_frontLeftSwerveModule.getSteerMotor().set(m_frontLeftSwerveModule.getPIDController().calculate(m_frontLeftSwerveModule.getCANCoder().getAbsolutePosition().getValueAsDouble()));
    m_frontRightSwerveModule.getSteerMotor().set(m_frontRightSwerveModule.getPIDController().calculate(m_frontRightSwerveModule.getCANCoder().getAbsolutePosition().getValueAsDouble()));
    m_backLeftSwerveModule.getSteerMotor().set(m_backLeftSwerveModule.getPIDController().calculate(m_backLeftSwerveModule.getCANCoder().getAbsolutePosition().getValueAsDouble()));
    m_backRightSwerveModule.getSteerMotor().set(m_backRightSwerveModule.getPIDController().calculate(m_backRightSwerveModule.getCANCoder().getAbsolutePosition().getValueAsDouble()));
    
  
  }


  public SwerveModule getFrontLeftSwerveModule() {
    return this.m_frontLeftSwerveModule;
  }

  public SwerveModule getFrontRightSwerveModule() {
    return this.m_frontRightSwerveModule;
  }

  public SwerveModule getBackLeftSwerveModule() {
    return this.m_backLeftSwerveModule;
  }

  public SwerveModule getBackRightSwerveModule() {
    return this.m_backRightSwerveModule;
  }
}
