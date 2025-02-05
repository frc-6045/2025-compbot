package frc.robot.commands.ElevatorCommands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class HoldElevator extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private double position;

  /**
   * Creates a new TestSparkFlex.
   *
   * @param subsystem The subsystem used by this command.
   */
  public HoldElevator(ElevatorSubsystem elevator) {
    m_ElevatorSubsystem = elevator;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position = m_ElevatorSubsystem.getRelativeEncoderPosition();
    System.out.println("holding elevator");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ElevatorSubsystem.goToSetpoint(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ElevatorSubsystem.stopElevatorMotors();
    System.out.println("holding elevator interrupt");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
