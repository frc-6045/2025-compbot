package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class PIDArmCommand extends Command {
    private PIDController m_ArmPIDController;
    private final ArmSubsystem m_ArmMotor;
    private double setPoint;

    public PIDArmCommand(ArmSubsystem m_ArmMotor, double setPoint) {
        this.m_ArmMotor = m_ArmMotor;
        this.setPoint = setPoint;

        m_ArmPIDController = new PIDController(4, 0, 0);
        m_ArmPIDController.enableContinuousInput(0, 1);
        //m_ArmPIDController.setTolerance(0.0038);
        addRequirements(m_ArmMotor);
    }

    @Override
    public void execute() {
        //double feedforward = 0.01;
        //if (m_ArmMotor.getAbsoluteEncoderPosition()-setPoint<0.01 && m_ArmMotor.getAbsoluteEncoderPosition()-setPoint>-0.01) m_ArmMotor.stopArmMotor();;;
        double speed = m_ArmPIDController.calculate(m_ArmMotor.getAbsoluteEncoderPosition(), setPoint);
        //speed = (speed>0) ? speed + feedforward : speed-feedforward;
        m_ArmMotor.setSpeed(speed);
        System.out.println("PIDArm output (speed): " + speed + "\nset point: " + m_ArmPIDController.getSetpoint() + "\ncurrent position: " + m_ArmMotor.getAbsoluteEncoderPosition());
    }

    @Override
    public void end(boolean Interrupted) {
        m_ArmMotor.stopArmMotor();
    }
}
