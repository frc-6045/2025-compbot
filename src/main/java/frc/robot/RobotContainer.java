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
import frc.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PIDArmAndElevator;
import frc.robot.commands.StopPIDArmAndElevator;
import frc.robot.commands.ArmCommands.ArmCommand;
import frc.robot.commands.ArmCommands.HoldArm;
import frc.robot.commands.ArmCommands.PIDArmCommand;
import frc.robot.commands.ArmCommands.StopPIDArmCommand;
import frc.robot.commands.ElevatorCommands.ElevatorCommand;
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
  public static final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  public static final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  public static int BumperPressed = 0;
  
  public final SwerveSubsystem m_DriveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  // define controllers
  private static final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final CommandXboxController m_driverController = 
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

    /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_DriveSubsystem.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(m_driverController::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                             m_driverController::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(m_DriveSubsystem.getSwerveDrive(),
                                                                        () -> -m_driverController.getLeftY(),
                                                                        () -> -m_driverController.getLeftX())
                                                                    .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *2))
                                                                               .headingWhile(true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureDrivetrain();
    m_ArmSubsystem.setDefaultCommand(new HoldArm(m_ArmSubsystem));
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureDrivetrain() {
    Command driveFieldOrientedDirectAngle      = m_DriveSubsystem.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = m_DriveSubsystem.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = m_DriveSubsystem.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = m_DriveSubsystem.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = m_DriveSubsystem.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = m_DriveSubsystem.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = m_DriveSubsystem.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      m_DriveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      m_DriveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      m_driverController.start().onTrue(Commands.runOnce(() -> m_DriveSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      m_driverController.button(1).whileTrue(m_DriveSubsystem.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      m_DriveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      m_driverController.x().whileTrue(Commands.runOnce(m_DriveSubsystem::lock, m_DriveSubsystem).repeatedly());
      m_driverController.y().whileTrue(m_DriveSubsystem.driveToDistanceCommand(1.0, 0.2));
      m_driverController.start().onTrue((Commands.runOnce(m_DriveSubsystem::zeroGyro)));
      m_driverController.back().whileTrue(m_DriveSubsystem.centerModulesCommand());
      m_driverController.leftBumper().onTrue(Commands.none());
      m_driverController.rightBumper().onTrue(Commands.none());
    } else
    {
      m_driverController.a().onTrue((Commands.runOnce(m_DriveSubsystem::zeroGyro)));
      m_driverController.x().onTrue(Commands.runOnce(m_DriveSubsystem::addFakeVisionReading));
      m_driverController.b().whileTrue(
          m_DriveSubsystem.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      m_driverController.start().whileTrue(Commands.none());
      m_driverController.back().whileTrue(Commands.none());
      m_driverController.leftBumper().whileTrue(Commands.runOnce(m_DriveSubsystem::lock, m_DriveSubsystem).repeatedly());
      m_driverController.rightBumper().onTrue(Commands.none());
    }
  }

  private void configureBindings() {
    // Gyro Heading Reset
    //m_driverController.start().onTrue(new InstantCommand(() -> {m_DriveSubsystem.zeroHeading();}, m_DriveSubsystem));

    // operator or driver triggers control coral intake
    m_operatorController.leftTrigger().whileTrue(new IntakeCommand(m_IntakeSubsystem, m_operatorController));
    m_operatorController.rightTrigger().whileTrue(new IntakeCommand(m_IntakeSubsystem, m_operatorController));
    //m_driverController.leftTrigger().whileTrue(new IntakeCommand(m_IntakeSubsystem, m_driverController));
    //m_driverController.rightTrigger().whileTrue(new IntakeCommand(m_IntakeSubsystem, m_driverController));

    // arm
    //m_driverController.rightTrigger().whileTrue(new ArmCommand(m_ArmSubsystem, true, m_driverController));
    //m_driverController.leftTrigger().whileTrue(new ArmCommand(m_ArmSubsystem, false, m_driverController));
    
    m_operatorController.b().onTrue(new InstantCommand(() -> {System.out.println("\narm encoder value: " + m_ArmSubsystem.getAbsoluteEncoderPosition()); System.out.println("elev encoder value: " + m_ElevatorSubsystem.getRelativeEncoderPosition());}));
    m_operatorController.x().onTrue(new StopPIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem)); // stop PID arm and elevator

    // SETPOINTS FOR OPERATOR

    // Left Stick Forward -- L4  0.9316917657852173, 98.78194427490234 (max)
    // Left Stick Left -- L1 0.25360676646232605, 0
    // Left Stick Right -- L2 11111111122333131313131313113133
    // Left Stick Down -- L3 0.967854917049408, 0
    // Coral Station -- A 0.4507419764995575, 47.08803176879883

    m_operatorController.y().onTrue(new PIDArmAndElevator(m_ArmSubsystem, 0.32972583174705505, m_ElevatorSubsystem, 14));

    m_operatorController.a().onTrue(new PIDArmAndElevator(m_ArmSubsystem, 0.4507056772708893, m_ElevatorSubsystem, 44.03850555419922));

    //m_operatorController.leftStick().onTrue(new PIDArmAndElevator(m_ArmSubsystem, 0.25360676646232605, m_ElevatorSubsystem, 0));
    //m_operatorController.pov(270).onTrue(new PIDArmAndElevator(m_ArmSubsystem, 0.5, m_ElevatorSubsystem, 0));
    
  //m_operatorController.rightStick().onTrue(new PIDArmAndElevator(m_ArmSubsystem, 0.967854917049408, m_ElevatorSubsystem, 0));
 // m_operatorController.getLeftY().
    m_operatorController.leftStick().onTrue(new PIDArmAndElevator(m_ArmSubsystem, 0.9521994590759277, m_ElevatorSubsystem, 101.27217864990234));
    m_operatorController.rightStick().onTrue(new PIDArmAndElevator(m_ArmSubsystem, 0.9638405442237854, m_ElevatorSubsystem, 2.305943012237549));


    //Quinn's Crap

    //Grant Changed this to be on bumpers instead of dpad
    m_driverController.rightTrigger().whileTrue(new ArmCommand(m_ArmSubsystem, true, m_driverController));
    m_driverController.leftTrigger().whileTrue(new ArmCommand(m_ArmSubsystem, false, m_driverController));
    //m_operatorController.pov(90).whileTrue(new ArmCommand(m_ArmSubsystem, true, m_operatorController));
    //m_operatorController.pov(270).whileTrue(new ArmCommand(m_ArmSubsystem, false, m_operatorController));
    
    // d pad controls elevator
    m_operatorController.pov(0).whileTrue(new ElevatorCommand(m_ElevatorSubsystem, true));
    m_operatorController.pov(180).whileTrue(new ElevatorCommand(m_ElevatorSubsystem, false));


  }
  // /** 
  //  * Use this to pass the autonomous command to the main {@link Robot} class.
  //  *
  //  * @return the command to run in autonomous
  //  */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //  // return Autos.exampleAuto(m_sparkFlexTesterSubsystem);
  // }
}
