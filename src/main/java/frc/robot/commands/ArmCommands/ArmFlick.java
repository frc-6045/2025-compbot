package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmFlick extends Command {
    private final ArmSubsystem m_ArmSubsystem;
    private double initialPosition;
    private boolean state;

    public ArmFlick(ArmSubsystem m_ArmSubsystem) {
        this.m_ArmSubsystem = m_ArmSubsystem;

        addRequirements(m_ArmSubsystem);
    }

    @Override
    public void initialize() {
        initialPosition = m_ArmSubsystem.getAbsoluteEncoderPosition();
        state=true;
        System.out.println("aaaaaaa");
  }

    @Override
    public void execute() {
        //double feedforward = 0.01;
        System.out.println("desired pos: " + (initialPosition+0.05) + 
                            "\ncurrent pos: " + m_ArmSubsystem.getAbsoluteEncoderPosition() +
                            "\n" + m_ArmSubsystem.atSetpoint());
        if (state) {
            m_ArmSubsystem.goToSetpoint(initialPosition+PositionConstants.kArmFlickDistance1);
            if (m_ArmSubsystem.atSetpoint()) state=false;
        }
        else {
            m_ArmSubsystem.goToSetpoint(initialPosition+PositionConstants.kArmFlickDistance2);
        }

    }

    @Override
    public void end(boolean Interrupted) {
        m_ArmSubsystem.stopArmMotor();
    }
}