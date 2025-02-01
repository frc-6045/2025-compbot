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

import javax.swing.text.Position;

import edu.wpi.first.math.MathUtil;
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
  public final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();

  // define controllers
  private static final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final CommandXboxController m_driverController = 
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_ArmSubsystem.setDefaultCommand(new HoldArm(m_ArmSubsystem));
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

  public static void buttonStuff() {
    if (m_operatorController.getLeftY() == -1) new RunCommand(()->{new PIDArmAndElevator(m_ArmSubsystem, 0.25360676646232605, m_ElevatorSubsystem, 0);});
    if (m_operatorController.getLeftY() == 1) new PIDArmAndElevator(m_ArmSubsystem, 0.5, m_ElevatorSubsystem, 0);
    if (m_operatorController.getLeftX() == -1) new PIDArmAndElevator(m_ArmSubsystem, 0.967854917049408, m_ElevatorSubsystem, 0);
    if (m_operatorController.getLeftX() == 1) new PIDArmAndElevator(m_ArmSubsystem, 0.9316917657852173, m_ElevatorSubsystem, 98.78194427490234);
  } 
  private void configureBindings() {
    // Gyro Heading Reset
    m_driverController.start().onTrue(new InstantCommand(() -> {m_DriveSubsystem.zeroHeading();}, m_DriveSubsystem));

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
    //m_operatorController.leftStick().whileTrue(new InstantCommand(() -> { System.out.println(m_operatorController.getLeftY());
    //  if (m_operatorController.getLeftY() == -1) System.out.println("hi"); //new PIDArmAndElevator(m_ArmSubsystem, 0.25360676646232605, m_ElevatorSubsystem, 0);
    //  if (m_operatorController.getLeftY() == 1) new PIDArmAndElevator(m_ArmSubsystem, 0.5, m_ElevatorSubsystem, 0);
    //  if (m_operatorController.getLeftX() == -1) new PIDArmAndElevator(m_ArmSubsystem, 0.967854917049408, m_ElevatorSubsystem, 0);
    //  if (m_operatorController.getLeftX() == 1) new PIDArmAndElevator(m_ArmSubsystem, 0.9316917657852173, m_ElevatorSubsystem, 98.78194427490234);
    //}));

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
    //Grant Changed this to be on x/y instead of dpad
    m_operatorController.pov(0).whileTrue(new ElevatorCommand(m_ElevatorSubsystem, true));
    m_operatorController.pov(180).whileTrue(new ElevatorCommand(m_ElevatorSubsystem, false));
    //m_operatorController.y().whileTrue(new ElevatorCommand(m_ElevatorSubsystem, true));
    //m_operatorController.a().whileTrue(new ElevatorCommand(m_ElevatorSubsystem, false));
      
    // paddles will have setpoints 1-8

    m_DriveSubsystem.setDefaultCommand(
    new RunCommand(
          () -> m_DriveSubsystem.drive( 
              MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.30), 
              MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.30),
              MathUtil.applyDeadband(-m_driverController.getRightX(), 0.30),
              true),
          m_DriveSubsystem)
    );

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
