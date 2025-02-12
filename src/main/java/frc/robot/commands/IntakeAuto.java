package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.MotorConstants;
import frc.robot.RobotContainer;

public class IntakeAuto extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final double time;
    private final Timer timer = new Timer();
    
    public IntakeAuto(IntakeSubsystem intakeSubsystem, double timee) {
        m_IntakeSubsystem = intakeSubsystem;
        time = timee;
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        System.out.println("auto pls work!");
    }

    @Override
    public void execute() {
        if (timer.get() < time) {
            m_IntakeSubsystem.setSpeed(0.2, 1, 1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.setSpeed(0, 0, 0);    
    }

}
