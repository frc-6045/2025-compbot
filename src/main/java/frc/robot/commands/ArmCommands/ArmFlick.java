package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmFlick extends Command {
    private final ArmSubsystem m_ArmSubsystem;
    private double initialPosition;
    private boolean state = false;

    public ArmFlick(ArmSubsystem m_ArmSubsystem) {
        this.m_ArmSubsystem = m_ArmSubsystem;

        addRequirements(m_ArmSubsystem);
    }

    @Override
    public void initialize() {
        initialPosition = m_ArmSubsystem.getAbsoluteEncoderPosition();
        System.out.println("aaaaaaa");
  }

    @Override
    public void execute() {
        //double feedforward = 0.01;
        if (!state) {
            System.out.println("initial pos: " + initialPosition + 
                            "\ncurrent pos: " + m_ArmSubsystem.getAbsoluteEncoderPosition() +
                            "\n" + m_ArmSubsystem.atSetpoint());
            m_ArmSubsystem.goToSetpoint(initialPosition+0.05);
            if (m_ArmSubsystem.atSetpoint()) 
            state=true;

        }
        else {
            m_ArmSubsystem.goToSetpoint(initialPosition-0.05);
        }

    }

    @Override
    public void end(boolean Interrupted) {
        m_ArmSubsystem.stopArmMotor();
    }
}