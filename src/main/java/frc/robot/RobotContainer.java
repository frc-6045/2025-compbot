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
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.subsystems.swerve.DriveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.commands.IntakeAuto;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PIDArmAndElevator;
import frc.robot.commands.StopPIDArmAndElevator;
import frc.robot.commands.ArmCommands.ArmCommand;
import frc.robot.commands.ArmCommands.HoldArm;
import frc.robot.commands.ElevatorCommands.ElevatorCommand;
import frc.robot.commands.ElevatorCommands.HoldElevator;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.File;

import javax.swing.text.Position;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private Autos m_Autos;
  
  public final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();

  // define controllers
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final CommandXboxController m_driverController = 
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_godController =
      new CommandXboxController(OperatorConstants.kGodControllerPort);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("coralSpit", new IntakeAuto(m_IntakeSubsystem, 0.222, true));
    NamedCommands.registerCommand("coralIntake", new IntakeAuto(m_IntakeSubsystem, 0.011, false));
    NamedCommands.registerCommand("CoralL1", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL1ArmPosition, m_ElevatorSubsystem, PositionConstants.kL1ElevatorPosition));
    NamedCommands.registerCommand("CoralL2", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL2ArmPosition, m_ElevatorSubsystem, PositionConstants.kL2ElevatorPosition));
    NamedCommands.registerCommand("CoralL3", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL3ArmPosition, m_ElevatorSubsystem, PositionConstants.kL3ElevatorPosition));
    NamedCommands.registerCommand("CoralL4", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL4ArmPosition, m_ElevatorSubsystem, PositionConstants.kL4ElevatorPosition));
    NamedCommands.registerCommand("CoralIntake", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kHumanArmPosition, m_ElevatorSubsystem, PositionConstants.kHumanElevatorPosition));
    NamedCommands.registerCommand("HomePosition", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kHomeArmPosition, m_ElevatorSubsystem, PositionConstants.kHomeElevatorPosition));
    m_Autos = new Autos(m_DriveSubsystem, m_IntakeSubsystem, m_ElevatorSubsystem, m_ArmSubsystem);
    
    Bindings.InitBindings(m_operatorController, m_driverController, m_godController, m_DriveSubsystem, m_ArmSubsystem, m_ElevatorSubsystem, m_IntakeSubsystem);
    
    m_DriveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> m_DriveSubsystem.drive( 
                MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.30)+MathUtil.applyDeadband(-m_godController.getLeftY(), 0.30), 
                MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.30)+MathUtil.applyDeadband(-m_godController.getLeftX(), 0.30),
                MathUtil.applyDeadband(-m_driverController.getRightX(), 0.30)+MathUtil.applyDeadband(-m_godController.getRightX(), 0.30),
                true),
            m_DriveSubsystem)
        );
    m_ArmSubsystem.setDefaultCommand(new HoldArm(m_ArmSubsystem));
    //m_ElevatorSubsystem.setDefaultCommand(new HoldElevator(m_ElevatorSubsystem));
    
    DriverStation.silenceJoystickConnectionWarning(true);
    
    
  }
  
  /** 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_Autos.getAutonomousCommand();
  }
}
