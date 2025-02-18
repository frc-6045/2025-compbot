package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

/** This will cancel whatever command is using the arm subsystem, which is probably just PID. */
public class StopPIDArmCommand extends InstantCommand {
    private final ArmSubsystem m_ArmMotor;
    public StopPIDArmCommand(ArmSubsystem armMotor) {
        m_ArmMotor=armMotor;
        addRequirements(m_ArmMotor);
    }

    @Override
    public void initialize() {m_ArmMotor.stopArmMotor();}
}
