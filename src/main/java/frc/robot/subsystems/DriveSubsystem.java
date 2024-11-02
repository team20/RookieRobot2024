// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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
  private SwerveDriveOdometry m_odometry;
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
    m_odometry = new SwerveDriveOdometry(
      m_kinematics, m_gyro.getRotation2d().unaryMinus(), getSwerveModulePositions()
    );
  }

  private SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[]{
      m_frontLeftSwerveModule.getPosition(),
      m_frontRightSwerveModule.getPosition(),
      m_backLeftSwerveModule.getPosition(),
      m_backRightSwerveModule.getPosition(),
    };
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

    m_frontLeftSwerveModule.updatePID();
    m_frontRightSwerveModule.updatePID();
    m_backLeftSwerveModule.updatePID();
    m_backRightSwerveModule.updatePID();
    m_odometry.update(m_gyro.getRotation2d().unaryMinus(), getSwerveModulePositions());
  }

  public void bindButtons(DoubleSupplier xAxisDrive, DoubleSupplier yAxisDrive, DoubleSupplier rotationAxis) {
    setDefaultCommand(run(() -> {
      // UPDATED to use wpilib swerve calculations
      // Get the foward, strafe, and rotation speed, using a deadband on the joystick
      // input so slight movements don't move the robot
      double fwdSpeed = MathUtil.applyDeadband(yAxisDrive.getAsDouble(), ControllerConstants.kDeadzone);
      double strSpeed = MathUtil.applyDeadband(xAxisDrive.getAsDouble(), ControllerConstants.kDeadzone);
      double rotSpeed = MathUtil.applyDeadband(rotationAxis.getAsDouble(), ControllerConstants.kDeadzone);

      setSpeeds(fwdSpeed, strSpeed, rotSpeed);
    }));
  }

  private void setSpeeds(double fwdSpeed, double strSpeed, double rotSpeed) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        fwdSpeed, strSpeed, rotSpeed, m_gyro.getRotation2d().unaryMinus());

      SmartDashboard.putString("Speeds", speeds.toString());

      // Now use this in our kinematics
      SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

      setSwerveStates(moduleStates);

      // SmartDashboard logging
      SmartDashboard.putNumber("Foward Speed", fwdSpeed);
      SmartDashboard.putNumber("Strafe Speed", strSpeed);
      SmartDashboard.putNumber("Rotation Speed", rotSpeed);
  }

  public Command resetHeadingCommand() {
    return runOnce(() -> m_gyro.zeroYaw());
  }

  public Command autoCommand(Pose2d goal) {
    return run(() -> {
      Transform2d direction = goal.minus(m_odometry.getPoseMeters());
      double fwdSpeed = Math.max(Math.min(direction.getX(), 1), -1);
      double strSpeed = Math.max(Math.min(direction.getY(), 1), -1);
      double rotSpeed = Math.max(Math.min(direction.getRotation().getDegrees(), 1), -1);
      setSpeeds(fwdSpeed, strSpeed, rotSpeed);
    }).until(() -> {
      Transform2d direction = goal.minus(m_odometry.getPoseMeters());
      return Math.abs(direction.getRotation().getDegrees()) < 2 && direction.getTranslation().getNorm() < 0.1;
    }).finallyDo(() -> {
      setSwerveStates(m_kinematics.toSwerveModuleStates(new ChassisSpeeds()));
    });
  }
}
