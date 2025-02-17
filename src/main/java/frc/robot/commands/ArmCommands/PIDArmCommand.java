package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class PIDArmCommand extends Command {
    private final ArmSubsystem m_ArmSubsystem;
    private double setPoint;
    private Timer timer;

    public PIDArmCommand(ArmSubsystem m_ArmSubsystem, double setPoint) {
        this.m_ArmSubsystem = m_ArmSubsystem;
        this.setPoint = setPoint;

        //m_ArmPIDController.setTolerance(0.0038);
        addRequirements(m_ArmSubsystem);
    }

    @Override
    public void execute() {
        //double feedforward = 0.01;
        m_ArmSubsystem.goToSetpoint(setPoint);
    }

    @Override
    public void end(boolean Interrupted) {
        m_ArmSubsystem.stopArmMotor();
        System.out.println("owo");
    }

    @Override
    public boolean isFinished() {
        //if (m_ArmSubsystem.atSetpoint()) return true;
        return false;
    }
}
