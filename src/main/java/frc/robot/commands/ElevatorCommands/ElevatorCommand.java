package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command { 
    private final ElevatorSubsystem m_elevatorSubsystem;
    private boolean goUp;
    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, boolean up) {
    m_elevatorSubsystem = elevatorSubsystem;
    goUp = up;
    
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = MotorConstants.kSparkFlexElevatorMotorsSpeed;
    System.out.println("open loop ELEVATOR " + m_elevatorSubsystem.getRelativeEncoderPosition() + " top: " + m_elevatorSubsystem.getTopLimitSwitchState() + " bottom: " + m_elevatorSubsystem.getBottomLimitSwitchState());
    if (goUp) {
        m_elevatorSubsystem.setSpeed(speed);
    } else {
        m_elevatorSubsystem.setSpeed(-1*speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.stopElevatorMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
