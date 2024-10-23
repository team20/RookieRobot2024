// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.atomic.DoubleAdder;
import java.util.function.DoubleSupplier;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  private Translation2d m_frontLeftLocation = new Translation2d(0.3, 0.3);
  private Translation2d m_frontRightLocation = new Translation2d(0.3, -0.3);
  private Translation2d m_backLeftLocation = new Translation2d(-0.3, 0.3);
  private Translation2d m_backRightLocation = new Translation2d(-0.3, -0.3);
  private AHRS m_gyro = new AHRS(SPI.Port.kMXP); 

  public static enum Operation {
    CMD_ANGLE, CMD_DISTANCE
  }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Initialize modules
    {
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
        DriveConstants.kFrontRightDriveInverted);

      m_backLeftSwerveModule = new SwerveModule(
        DriveConstants.kBackLeftCANCoderPort, 
        DriveConstants.kBackLeftDrivePort, 
        DriveConstants.kBackLeftSteerPort, 
        DriveConstants.kBackLeftDriveInverted);

      m_backRightSwerveModule = new SwerveModule(
        DriveConstants.kBackRightCANCoderPort, 
        DriveConstants.kBackRightDrivePort, 
        DriveConstants.kBackRightSteerPort, 
        DriveConstants.kBackRightDriveInverted);
    }
    // Creating my kinematics object using the module locations
    m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );
    m_gyro.zeroYaw();
    new Thread(() -> {
        try{
          Thread.sleep(1000);
          m_gyro.reset();
        }catch(Exception e){}
    });
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

    m_frontLeftSwerveModule.getSteerMotor().set(m_frontLeftSwerveModule.getPIDController().calculate(m_frontLeftSwerveModule.getSteerAngle()));
    m_frontRightSwerveModule.getSteerMotor().set(m_frontRightSwerveModule.getPIDController().calculate(m_frontRightSwerveModule.getSteerAngle()));
    m_backLeftSwerveModule.getSteerMotor().set(m_backLeftSwerveModule.getPIDController().calculate(m_backLeftSwerveModule.getSteerAngle()));
    m_backRightSwerveModule.getSteerMotor().set(m_backRightSwerveModule.getPIDController().calculate(m_backRightSwerveModule.getSteerAngle()));
  }

  public void bindButtons(DoubleSupplier xAxisDrive, DoubleSupplier yAxisDrive, DoubleSupplier rotationAxis) {
    setDefaultCommand(run(() -> {
      // UPDATED to use wpilib swerve calculations
      // Get the foward, strafe, and rotation speed, using a deadband on the joystick
      // input so slight movements don't move the robot
      double fwdSpeed = MathUtil.applyDeadband(yAxisDrive.getAsDouble(), ControllerConstants.kDeadzone);
      double strSpeed = MathUtil.applyDeadband(xAxisDrive.getAsDouble(), ControllerConstants.kDeadzone);
      double rotSpeed = MathUtil.applyDeadband(rotationAxis.getAsDouble(), ControllerConstants.kDeadzone);

      ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        fwdSpeed, strSpeed, rotSpeed, Rotation2d.fromDegrees(-m_gyro.getYaw()));

      SmartDashboard.putString("Speeds", speeds.toString());

      // Now use this in our kinematics
      SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

      setSwerveStates(moduleStates);

      // SmartDashboard logging
      SmartDashboard.putNumber("Foward Speed", fwdSpeed);
      SmartDashboard.putNumber("Strafe Speed", strSpeed);
      SmartDashboard.putNumber("Rotation Speed", rotSpeed);
    }));
  }

  public Command resetHeadingCommand() {
    return run(() -> setSteerMotors(0, 0, 0, 0));
  }

  public Command autoCommand(Operation op, double amount) {
    DoubleAdder m_amount = new DoubleAdder();
    m_amount.add(amount);
    return run(() -> {
      // System.out.println(m_op == Operation.CMD_ANGLE);
      if (op == Operation.CMD_ANGLE) { 
        setSteerMotors(m_amount.sum(), m_amount.sum(), m_amount.sum(), m_amount.sum());
      } else {
        // turn on motors - set 20% power for now
        double m_powerLevel = 0.2;
        setDriveMotors(m_powerLevel, m_powerLevel, m_powerLevel, m_powerLevel);
      }
    }).beforeStarting(() -> {
      if (op == Operation.CMD_DISTANCE) {
        double currentPosition = (m_frontLeftSwerveModule.getDriveEncoder().getPosition());
        SmartDashboard.putNumber("current position", currentPosition);
        m_amount.add(currentPosition); 
      } else {
        SmartDashboard.putNumber("starting angle", m_frontLeftSwerveModule.getSteerAngle());
      }
    }).until(() -> {
      switch(op){
        case CMD_ANGLE:
          double encAng = m_frontLeftSwerveModule.getSteerAngle();
          double ang1 = encAng - 360;
          //The error between the actual angle and the target angle
          double diff1 = Math.abs(encAng - m_amount.sum());
          double diff2 = Math.abs(ang1 - m_amount.sum());
          boolean isDone = (Math.min(diff1, diff2) < 2); // 2 degree tolerance
          SmartDashboard.putNumber("fl_angle", m_frontLeftSwerveModule.getSteerAngle());
          SmartDashboard.putNumber("bl_angle", m_backLeftSwerveModule.getSteerAngle());
          SmartDashboard.putNumber("fr_angle", m_frontRightSwerveModule.getSteerAngle());
          SmartDashboard.putNumber("br_angle", m_backLeftSwerveModule.getSteerAngle());

          return isDone;
        case CMD_DISTANCE:
            //Determine whether the target distance has been reached
            double currentPosition = (m_frontLeftSwerveModule.getDriveEncoder().getPosition());
            SmartDashboard.putNumber("currentPostion", currentPosition);
            SmartDashboard.putNumber("m_amount", m_amount.sum());
            return (m_frontLeftSwerveModule.getDriveEncoder().getPosition() >= m_amount.sum());
      }
            
      return false;
    }).finallyDo(() -> {
      switch(op){
        case CMD_ANGLE:
          SmartDashboard.putNumber("CAC end", m_amount.sum());
          break;
        case CMD_DISTANCE:
          //Determine whether the target distance has been reached
          setDriveMotors(0,0,0,0);
      }
    });
  }
}
