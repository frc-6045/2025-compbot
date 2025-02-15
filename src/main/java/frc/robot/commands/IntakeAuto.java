package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAuto extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final double time;
    private final Timer timer = new Timer();
    private final boolean direction;
    
    public IntakeAuto(IntakeSubsystem intakeSubsystem, double timee, boolean direction) {
        m_IntakeSubsystem = intakeSubsystem;
        time = timee;
        addRequirements(m_IntakeSubsystem);
        this.direction=direction;
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
            double speed = direction ? 0.7 : -0.7;
            m_IntakeSubsystem.setSpeed(speed, 1, 1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.setSpeed(0, 0, 0);    
    }

}
