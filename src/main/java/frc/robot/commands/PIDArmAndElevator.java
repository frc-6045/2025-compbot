package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ArmCommands.PIDArmCommand;
import frc.robot.commands.ArmCommands.PIDArmThenHold;
import frc.robot.commands.ElevatorCommands.PIDElevatorCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class PIDArmAndElevator extends Command {
    private final ArmSubsystem m_ArmSubsystem;
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private final double armSetpoint;
    private final double elevatorSetPoint;
    private Timer timer;
    public PIDArmAndElevator(ArmSubsystem armSubsystem, double armSetPoint, ElevatorSubsystem elevatorSubsystem, double elevatorSetPoint) {
        m_ArmSubsystem = armSubsystem;
        m_ElevatorSubsystem = elevatorSubsystem;
        this.armSetpoint = armSetPoint;
        this.elevatorSetPoint = elevatorSetPoint;
        addRequirements(m_ArmSubsystem, m_ElevatorSubsystem);
    }
    
    @Override
    public void execute() {
        m_ArmSubsystem.goToSetpoint(armSetpoint);
        m_ElevatorSubsystem.goToSetpoint(elevatorSetPoint);
    }

    @Override
    public boolean isFinished() {
        if (m_ArmSubsystem.atSetpoint() && m_ElevatorSubsystem.atSetpoint()) {
            // if (m_ArmSubsystem.atSetpoint()) {timer.start();
            // System.out.println("timer start: " + timer.get());}
            // if (timer.get()>6) {
            //     return true;
            //     //System.out.println("return true owoowowowo");
            // }
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean Interrupted) {
        m_ArmSubsystem.stopArmMotor();
        m_ElevatorSubsystem.stopElevatorMotors();
    }

}
