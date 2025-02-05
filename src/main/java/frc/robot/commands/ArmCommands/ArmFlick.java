package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmFlick extends Command {
    private final ArmSubsystem m_ArmSubsystem;
    private final double initialPosition;
    private boolean state = false;

    public ArmFlick(ArmSubsystem m_ArmSubsystem) {
        this.m_ArmSubsystem = m_ArmSubsystem;
        initialPosition = m_ArmSubsystem.getAbsoluteEncoderPosition();

        addRequirements(m_ArmSubsystem);
    }

    @Override
    public void execute() {
        //double feedforward = 0.01;
        if (!state) {
            m_ArmSubsystem.goToSetpoint(initialPosition+0.10);
            if (m_ArmSubsystem.atSetpoint()) 
            state=true;
        }
         else
            m_ArmSubsystem.goToSetpoint(initialPosition-0.10);

    }

    @Override
    public void end(boolean Interrupted) {
        m_ArmSubsystem.stopArmMotor();
    }
}