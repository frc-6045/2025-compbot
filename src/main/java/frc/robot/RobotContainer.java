// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// constants
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PositionConstants;

// subsystems
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.commands.ArmCommands.HoldArm;
import frc.robot.commands.ElevatorCommands.HoldElevator;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  public static final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  public final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private Autos m_Autos;
  private Elastic.Notification notification = new Elastic.Notification();

  // define controllers
  private static final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final CommandXboxController m_driverController = 
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_godController =
      new CommandXboxController(OperatorConstants.kGodControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_Autos = new Autos(m_DriveSubsystem, m_IntakeSubsystem, m_ElevatorSubsystem, m_ArmSubsystem);
    NamedCommands.registerCommand("hello", Commands.print("hii"));
    Bindings.InitBindings(m_operatorController, m_driverController, m_godController, m_DriveSubsystem, m_ArmSubsystem, m_ElevatorSubsystem, m_IntakeSubsystem);
    m_ArmSubsystem.setDefaultCommand(new HoldArm(m_ArmSubsystem));
    //m_ElevatorSubsystem.setDefaultCommand(new HoldElevator(m_ElevatorSubsystem));
    DriverStation.silenceJoystickConnectionWarning(true);

    Elastic.sendNotification(notification
      .withLevel(NotificationLevel.INFO)
      .withTitle("hello")
      .withDescription("helllloooo!")
    . withDisplaySeconds(2.0)
    );
  }

  /** 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   return m_Autos.getAutonomousCommand();
  // }
}
