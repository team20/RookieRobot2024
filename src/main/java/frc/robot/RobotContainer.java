// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DevonPieInTheFaceWeMightExceedTheCharacterLimitForFilePathsSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PivotSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final GenericHID m_driveController = new GenericHID(ControllerConstants.kDriverControllerPort);
  private final GenericHID m_operatorController = new GenericHID(ControllerConstants.kOperatorControllerPort);
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final DevonPieInTheFaceWeMightExceedTheCharacterLimitForFilePathsSubsystem m_PIFSubsystem = new DevonPieInTheFaceWeMightExceedTheCharacterLimitForFilePathsSubsystem();
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
  // private final CounterWeightSubsystem m_counterWeightSubsystem = new CounterWeightSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    EventLoop loop = CommandScheduler.getInstance().getActiveButtonLoop();
    m_driveSubsystem.setDefaultDriveCommand(
      () -> m_driveController.getRawAxis(Axis.kLeftX),
      () -> m_driveController.getRawAxis(Axis.kLeftY),
      m_driveController.button(Button.kLeftBumper,loop),
      m_driveController.button(Button.kRightBumper,loop)
    );
    m_climberSubsystem.setDefaultClimberCommand(
      () -> m_operatorController.getRawAxis(Axis.kLeftY),
      () -> m_operatorController.getRawAxis(Axis.kRightY)
    );
    m_PIFSubsystem.setDefaultPIFCommand(
      m_operatorController.povUp(loop),
      m_operatorController.povDown(loop),
      m_operatorController.button(Button.kTriangle,loop),
      m_operatorController.button(Button.kX,loop),
      m_operatorController.button(Button.kCircle,loop),
      m_operatorController.button(Button.kSquare,loop)
    );
    m_pivotSubsystem.setDefaultPivotCommand(
      m_operatorController.button(Button.kLeftBumper,loop),
      m_operatorController.button(Button.kRightBumper,loop)
    );
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(m_driveSubsystem.autoAngleCommand(Rotation2d.fromDegrees(0)));
  }
}





