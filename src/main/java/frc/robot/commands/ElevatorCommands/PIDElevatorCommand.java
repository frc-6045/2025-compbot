package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class PIDElevatorCommand extends Command {
    private PIDController m_ElevatorPIDController;
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private double setPoint;

    public PIDElevatorCommand(ElevatorSubsystem elevatorSubsystem, double setPoint) {
        this.m_ElevatorSubsystem = elevatorSubsystem;
        this.setPoint = setPoint;

        m_ElevatorPIDController = new PIDController(6, 0, 0);
        //m_ElevatorPIDController.enableContinuousInput(0, 1);
        //m_ArmPIDController.setTolerance(0.0038);
        addRequirements(m_ElevatorSubsystem);
    }

    @Override
    public void execute() {
        //double feedforward = 0.01;
        //if (m_ArmMotor.getAbsoluteEncoderPosition()-setPoint<0.01 && m_ArmMotor.getAbsoluteEncoderPosition()-setPoint>-0.01) m_ArmMotor.stopArmMotor();;;
        double speed = -m_ElevatorPIDController.calculate(m_ElevatorSubsystem.getRelativeEncoderPosition(), setPoint);
        //speed = (speed>0) ? speed + feedforward : speed-feedforward;
        m_ElevatorSubsystem.setSpeed(speed);
        System.out.println("PIDElevator output (speed): " + speed + "\nset point: " + m_ElevatorPIDController.getSetpoint() + "\ncurrent position: " + m_ElevatorSubsystem.getRelativeEncoderPosition());
    }

    @Override
    public void end(boolean Interrupted) {
        m_ElevatorSubsystem.stopElevatorMotors();
    }
}
