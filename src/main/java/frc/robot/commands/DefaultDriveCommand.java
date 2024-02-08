// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Swerve drive joystick command
 * Based on
 * https://www.chiefdelphi.com/uploads/default/original/3X/8/c/8c0451987d09519712780ce18ce6755c21a0acc0.pdf
 * and
 * https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf
 */
public class DefaultDriveCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private Supplier<Double> m_yAxisDrive;
  private Supplier<Double> m_xAxisDrive;
  private Supplier<Double> m_rotationAxis;
  private double m_trackWidth;
  private double m_wheelBase;  
  private SwerveDriveKinematics m_kinematics;


  public DefaultDriveCommand(DriveSubsystem driveSubsystem, Supplier<Double> xAxisDrive, Supplier<Double> yAxisDrive,
      Supplier<Double> rotationAxis) {
    m_driveSubsystem = driveSubsystem;
    m_yAxisDrive = yAxisDrive;
    m_xAxisDrive = xAxisDrive;
    m_rotationAxis = rotationAxis;
    addRequirements(m_driveSubsystem);
  }

  @Override
  public void initialize() {
    m_trackWidth = DriveConstants.kTrackWidth;
    m_wheelBase = DriveConstants.kWheelBase;
        // Locations for the swerve drive modules relative to the robot center.
    Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    // Creating my kinematics object using the module locations
    m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );
  }

  /**
   * The main body of a command. Called repeatedly while the command is scheduled.
   * Takes joystick inputs, calculatees the wheel angles and speeds, moves the
   * robots with those values, and logs the calculated numbers
   */
  @Override
  public void execute() {
    // UPDATED to use wpilib swerve calculations
    // Get the foward, strafe, and rotation speed, using a deadband on the joystick
    // input so slight movements don't move the robot
    double fwdSpeed = MathUtil.applyDeadband(m_yAxisDrive.get(), ControllerConstants.kDeadzone);
    double strSpeed = MathUtil.applyDeadband(m_xAxisDrive.get(), ControllerConstants.kDeadzone);
    double rotSpeed = MathUtil.applyDeadband(m_rotationAxis.get(), ControllerConstants.kDeadzone);

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      fwdSpeed, strSpeed, rotSpeed, Rotation2d.fromDegrees(DriveSubsystem.get().getHeading()));

    // Now use this in our kinematics
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    DriveSubsystem.get().setSwerveStates(moduleStates);

    /*

    double leftStickMagnitude = Math.sqrt( fwdSpeed * fwdSpeed + strSpeed * strSpeed);
    double leftStickAngle = Math.atan2(strSpeed, fwdSpeed);
    
    // NavX returns gyro andle in degrees
    // Calculations below need radians
    double gyroAngleRad = Math.toRadians(m_driveSubsystem.getNavx().getYaw());

    // Rotation speed offsets for each wheel
    final double cos45 = Math.cos(Math.toRadians(45));
    final double cos135 = Math.cos(Math.toRadians(135));
    final double sin135 = Math.sin(Math.toRadians(135));
    final double sin225 = Math.sin(Math.toRadians(225));
    

    // The y component of the FL and BL wheels
    double b = (Math.abs(leftStickMagnitude) * Math.sin(leftStickAngle + gyroAngleRad) - rotSpeed * sin135);
    // The y component of the FR and BR wheels
    double a = (Math.abs(leftStickMagnitude) * Math.sin(leftStickAngle + gyroAngleRad) - rotSpeed * sin225);
    //The x component of the FL and FR
    double d = (leftStickMagnitude * Math.cos(leftStickAngle + gyroAngleRad) - rotSpeed * cos135);
    //The x component of the BL and BR
    double c = (leftStickMagnitude * Math.cos(leftStickAngle + gyroAngleRad) - rotSpeed * cos45);

    double FRONTLEFTX = (leftStickMagnitude * Math.cos(leftStickAngle + gyroAngleRad) + rotSpeed * cos135);
    double FRONTLEFTY = (Math.abs(leftStickMagnitude) * Math.sin(leftStickAngle + gyroAngleRad) + rotSpeed * sin135);
    double BACKRIGHTX = (leftStickMagnitude * Math.cos(leftStickAngle + gyroAngleRad) + rotSpeed * cos45);
    double BACKRIGHTY = (Math.abs(leftStickMagnitude) * Math.sin(leftStickAngle + gyroAngleRad) + rotSpeed * sin225);

    // Calculate the wheel speeds
    double frontLeftSpeed = Math.sqrt(b * b + d * d);
    double frontRightSpeed = Math.sqrt(b * b + c * c);
    double backLeftSpeed = Math.sqrt(a * a + d * d);
    double backRightSpeed = Math.sqrt(a * a + c * c);
    // Initalizing the highest speed, saves one if statement
    double highestSpeed = frontLeftSpeed;

    // Get max speed of all 4 wheels
    highestSpeed = Math.max(highestSpeed, Math.abs(frontRightSpeed));
    highestSpeed = Math.max(highestSpeed, Math.abs(backLeftSpeed));
    highestSpeed = Math.max(highestSpeed, Math.abs(backRightSpeed));

      // Normalize all speeds to the max speed so nothing is > 1
    if (highestSpeed > 0) {
      frontRightSpeed /= highestSpeed;
      frontLeftSpeed /= highestSpeed;
      backRightSpeed /= highestSpeed;
      backLeftSpeed /= highestSpeed;
    }
    
    double frontLeftAngle = Math.toDegrees(Math.atan2(FRONTLEFTY, FRONTLEFTX));
    double frontRightAngle = Math.toDegrees(Math.atan2(a, d));
    double backLeftAngle = Math.toDegrees(Math.atan2(b, c));
    double backRightAngle = Math.toDegrees(Math.atan2(BACKRIGHTY, BACKRIGHTX));

    
    // SmartDashboard logging
    {
      SmartDashboard.putNumber("Foward Speed", fwdSpeed);
      SmartDashboard.putNumber("Strafe Speed", strSpeed);
      SmartDashboard.putNumber("Rotation Speed", rotSpeed);
      SmartDashboard.putNumber("a", a);
      SmartDashboard.putNumber("b", b);
      SmartDashboard.putNumber("c", c);
      SmartDashboard.putNumber("d", d);
      SmartDashboard.putNumber("Front Right Wheel Speed", frontRightSpeed);
      SmartDashboard.putNumber("Front Left Wheel Speed", frontLeftSpeed);
      SmartDashboard.putNumber("Back Right Wheel Speed", backRightSpeed);
      SmartDashboard.putNumber("Back Left Wheel Speed", backLeftSpeed);
      SmartDashboard.putNumber("Front Right Wheel Angle", frontRightAngle);
      SmartDashboard.putNumber("Front Left Wheel Angle", frontLeftAngle);
      SmartDashboard.putNumber("Back Right Wheel Angle", backRightAngle);
      SmartDashboard.putNumber("Back Left Wheel Angle", backLeftAngle);
      SmartDashboard.putNumber("NavX yaw angle", Math.toDegrees(gyroAngleRad));

    }

    if(leftStickMagnitude + Math.abs(rotSpeed) > 0.1){
      // Move the robot
      m_driveSubsystem.setSteerMotors(frontLeftAngle, frontRightAngle, backLeftAngle, backRightAngle);
    }
    m_driveSubsystem.setDriveMotors(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    */
   }
}
