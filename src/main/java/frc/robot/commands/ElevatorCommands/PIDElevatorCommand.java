package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class PIDElevatorCommand extends Command {
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private double setpoint;

    public PIDElevatorCommand(ElevatorSubsystem elevatorSubsystem, double setPoint) {
        this.m_ElevatorSubsystem = elevatorSubsystem;
        setpoint = setPoint;

        
        addRequirements(m_ElevatorSubsystem);
    }

    @Override
    public void execute() {
        m_ElevatorSubsystem.goToSetpoint(setpoint);
    }

    @Override
    public void end(boolean Interrupted) {
        m_ElevatorSubsystem.stopElevatorMotors();
    }
}
