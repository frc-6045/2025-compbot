package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;

public class PIDArmThenHold extends SequentialCommandGroup {
    public PIDArmThenHold(ArmSubsystem arm, double setpoint) {
        addCommands(new PIDArmCommand(arm, setpoint), new HoldArm(arm));
    }
}
