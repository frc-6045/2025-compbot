package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ArmCommands.PIDArmCommand;
import frc.robot.commands.ElevatorCommands.PIDElevatorCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class PIDArmAndElevator extends ParallelCommandGroup {
    public PIDArmAndElevator(ArmSubsystem armSubsystem, double armSetPoint, ElevatorSubsystem elevatorSubsystem, double elevatorSetPoint) {
        addCommands(
            new PIDArmCommand(armSubsystem, armSetPoint),
            new PIDElevatorCommand(elevatorSubsystem, elevatorSetPoint)
        );
    }
}
