package frc.robot.commands.ArmCommands;

import frc.robot.Constants.MotorConstants;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class HoldArm extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_ArmSubsystem;
  private double position;

  /**
   * Creates a new TestSparkFlex.
   *
   * @param subsystem The subsystem used by this command.
   */
  public HoldArm(ArmSubsystem armMotor) {
    m_ArmSubsystem = armMotor;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position = m_ArmSubsystem.getAbsoluteEncoderPosition();
    System.out.println("holding arm");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ArmSubsystem.goToSetpoint(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.stopArmMotor();
    System.out.println("holding arm interrupt");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
