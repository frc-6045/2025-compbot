package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ArmCommands.StopPIDArmCommand;
import frc.robot.commands.ElevatorCommands.StopPIDElevatorCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/** Cancel any commands using arm and elevator subsystems. */
public class StopPIDArmAndElevator extends ParallelCommandGroup {
    public StopPIDArmAndElevator(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addCommands(
            new StopPIDArmCommand(armSubsystem),
            new StopPIDElevatorCommand(elevatorSubsystem)
        );
    }
}
