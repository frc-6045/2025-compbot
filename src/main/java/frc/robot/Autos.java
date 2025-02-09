package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class Autos {
    private final DriveSubsystem m_DriveSubsystem;
    private final IntakeSubsystem m_IntakeSubsystem;
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private final ArmSubsystem m_ArmSubsystem;
    private SendableChooser<Command> autoChooser;
    //private ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    public Autos(DriveSubsystem drive, IntakeSubsystem intake, ElevatorSubsystem elev, ArmSubsystem arm) {
        m_DriveSubsystem = drive;
        m_IntakeSubsystem = intake;
        m_ElevatorSubsystem = elev;
        m_ArmSubsystem = arm;
        autoChooser = new SendableChooser<Command>();
        autoChooser.addOption("DoNothing", new InstantCommand());
        autoChooser.addOption("DoNothing2", new InstantCommand());
        SmartDashboard.putData("autos", autoChooser);
        Shuffleboard.getTab("Test").add("test!!!",autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    // public Command getAutonomousCommand() {
    //     return autoChooser.getSelected();
    // }
}
