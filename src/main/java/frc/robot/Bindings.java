package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PIDArmAndElevator;
import frc.robot.commands.StopPIDArmAndElevator;
import frc.robot.commands.ArmCommands.ArmCommand;
import frc.robot.commands.ArmCommands.ArmFlick;
import frc.robot.commands.ElevatorCommands.ElevatorCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import swervelib.SwerveInputStream;

public class Bindings {
    public static void InitBindings(
        CommandXboxController m_operatorController, 
        CommandXboxController m_driverController, 
        CommandXboxController m_godController,
        DriveSubsystem m_driveSubsystem,
        ArmSubsystem m_ArmSubsystem, 
        ElevatorSubsystem m_ElevatorSubsystem, 
        IntakeSubsystem m_IntakeSubsystem) {

        // Gyro Heading Reset
        // m_driverController.start().onTrue(new InstantCommand(() -> {m_DriveSubsystem.zeroHeading();}, m_DriveSubsystem));

        // Operator Controller bindings
        m_operatorController.leftTrigger().whileTrue(new IntakeCommand(m_IntakeSubsystem, m_operatorController));
        m_operatorController.rightTrigger().whileTrue(new IntakeCommand(m_IntakeSubsystem, m_operatorController));

        m_operatorController.leftBumper().onTrue(new InstantCommand(() -> {System.out.println("\narm encoder value: " + m_ArmSubsystem.getAbsoluteEncoderPosition()); System.out.println("elev encoder value: " + m_ElevatorSubsystem.getRelativeEncoderPosition());}));
        m_operatorController.x().onTrue(new StopPIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem)); // stop PID arm and elevator
        
        m_operatorController.pov(270).onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL2ArmPosition, m_ElevatorSubsystem, PositionConstants.kL2ElevatorPosition));
        m_operatorController.b().onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL1ArmPosition, m_ElevatorSubsystem, PositionConstants.kL1ElevatorPosition));
        m_operatorController.y().onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kHomeArmPosition, m_ElevatorSubsystem, PositionConstants.kHomeElevatorPosition));
        m_operatorController.a().onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kHumanArmPosition, m_ElevatorSubsystem, PositionConstants.kHumanElevatorPosition));
        m_operatorController.leftStick().onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL3ArmPosition, m_ElevatorSubsystem, PositionConstants.kL3ElevatorPosition));
        m_operatorController.rightStick().onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL4ArmPosition, m_ElevatorSubsystem, PositionConstants.kL4ElevatorPosition));
        
        m_operatorController.rightBumper().onTrue(new ArmFlick(m_ArmSubsystem));

        m_operatorController.pov(0).whileTrue(new ElevatorCommand(m_ElevatorSubsystem, true));
        m_operatorController.pov(180).whileTrue(new ElevatorCommand(m_ElevatorSubsystem, false));

        // Driver Controller bindings
        m_driverController.leftTrigger().whileTrue(new IntakeCommand(m_IntakeSubsystem, m_driverController));
        m_driverController.rightTrigger().whileTrue(new IntakeCommand(m_IntakeSubsystem, m_driverController));

        m_driverController.rightBumper().whileTrue(new ArmCommand(m_ArmSubsystem, true, m_driverController));
        m_driverController.leftBumper().whileTrue(new ArmCommand(m_ArmSubsystem, false, m_driverController));

        // Controller with both driver and operator functions
        m_godController.leftTrigger().whileTrue(new IntakeCommand(m_IntakeSubsystem, m_operatorController));
        m_godController.rightTrigger().whileTrue(new IntakeCommand(m_IntakeSubsystem, m_operatorController));

        m_godController.rightBumper().whileTrue(new ArmCommand(m_ArmSubsystem, true, m_driverController));
        m_godController.leftBumper().whileTrue(new ArmCommand(m_ArmSubsystem, false, m_driverController));

        m_godController.b().onTrue(new InstantCommand(() -> {System.out.println("\narm encoder value: " + m_ArmSubsystem.getAbsoluteEncoderPosition()); System.out.println("elev encoder value: " + m_ElevatorSubsystem.getRelativeEncoderPosition());}));
        m_godController.x().onTrue(new StopPIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem)); // stop PID arm and elevator
        
        m_godController.y().onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kHomeArmPosition, m_ElevatorSubsystem, PositionConstants.kHomeElevatorPosition));
        m_godController.a().onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kHumanArmPosition, m_ElevatorSubsystem, PositionConstants.kHumanElevatorPosition));
        m_godController.pov(270).onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL3ArmPosition, m_ElevatorSubsystem, PositionConstants.kL3ElevatorPosition));
        m_godController.pov(90).onTrue(new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL4ArmPosition, m_ElevatorSubsystem, PositionConstants.kL4ElevatorPosition));
    }

}
