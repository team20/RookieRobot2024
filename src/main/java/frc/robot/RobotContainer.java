// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Random;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final GenericHID m_driverController = new GenericHID(ControllerConstants.kDriverControllerPort);
  private final GenericHID m_operatorController = new GenericHID(ControllerConstants.kOperatorControllerPort);
  private final SendableChooser<Supplier<Command>> m_autoSelector = new SendableChooser<>();
  private final SendableChooser<Integer> m_distanceSelector = new SendableChooser<>();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final TransportSubsystem m_transportSubsystem = new TransportSubsystem();
  // private final CounterWeightSubsystem m_counterWeightSubsystem = new CounterWeightSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings 
    configureButtonBindings();
    // Auto chooser
    m_distanceSelector.addOption("1", 1);
    m_distanceSelector.addOption("5", 5);
    m_distanceSelector.addOption("10", 10);
    m_distanceSelector.addOption("50", 50);
    m_distanceSelector.addOption("100", 100);
    SmartDashboard.putData(m_distanceSelector);
    m_autoSelector.addOption("Go forward", () -> m_driveSubsystem.autoCommand(
      new Pose2d(m_distanceSelector.getSelected(), 0, new Rotation2d())
    ));
    m_autoSelector.addOption("Spin", () -> m_driveSubsystem.autoCommand(
      new Pose2d(0, 0, Rotation2d.fromRotations(1))
    ));
    Random r = new Random();
    m_autoSelector.addOption("Luck",
      () -> Commands.waitUntil(() -> r.nextDouble() < 0.000001)
      .andThen(Commands.print("You got lucky!"))
    );
    m_autoSelector.addOption("Idle", () -> Commands.idle());
    SmartDashboard.putData(m_autoSelector);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    EventLoop loop = CommandScheduler.getInstance().getActiveButtonLoop();
    m_driveSubsystem.bindButtons(
      () -> m_driverController.getRawAxis(Axis.kLeftX ),
      () -> m_driverController.getRawAxis(Axis.kLeftY ),
      () -> m_driverController.getRawAxis(Axis.kRightX)
    );
    m_intakeSubsystem.bindButtons(
      m_operatorController.povUp(  loop),
      m_operatorController.povDown(loop)
    );
    m_pivotSubsystem.bindButtons(
      () -> m_operatorController.getRawAxis(Axis.kLeftTrigger),
      () -> m_operatorController.getRawAxis(Axis.kRightTrigger)
    );
    m_climberSubsystem.bindButtons(
      () -> m_operatorController.getRawAxis(Axis.kRightY)
    );
    m_shooterSubsystem.bindButtons(
      m_driverController  .button(Button.kSquare,   loop),
      m_operatorController.button(Button.kTriangle, loop),
      m_operatorController.button(Button.kX,        loop)
    );
    m_transportSubsystem.bindButtons(
      m_operatorController.button(Button.kRightBumper, loop),
      m_operatorController.button(Button.kLeftBumper,  loop)
    );
    new Trigger(m_driverController.button(Button.kRightBumper, loop)).onTrue(m_driveSubsystem.resetHeadingCommand());
  }

  public Command getAutonomousCommand() {
    return m_autoSelector.getSelected().get();
  }
}
