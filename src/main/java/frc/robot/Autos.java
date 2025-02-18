package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Autos {
    // we probably don't need these subsystems here, but I'll leave them for now
    private final SwerveSubsystem m_DriveSubsystem;
    private final IntakeSubsystem m_IntakeSubsystem;
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private final ArmSubsystem m_ArmSubsystem;
    private SendableChooser<Command> autoChooser;

    public Autos(SwerveSubsystem drive, IntakeSubsystem intake, ElevatorSubsystem elev, ArmSubsystem arm) {
        m_IntakeSubsystem = intake;
        m_ElevatorSubsystem = elev;
        m_ArmSubsystem = arm;
        m_DriveSubsystem = drive;

        //incorprate pathplanner into code
        autoChooser = new SendableChooser<Command>();
        autoChooser.addOption("SetpointThenIntake", AutoBuilder.buildAuto("SetpointThenIntake"));
        autoChooser.addOption("3piece", AutoBuilder.buildAuto("3PieceJKLTesting"));
        autoChooser.addOption("BackStart2Coral", AutoBuilder.buildAuto("BackStart2Coral"));
        autoChooser.addOption("3PieceJKLPolesWithCommands", AutoBuilder.buildAuto("3PieceJKLPoles"));
        autoChooser.addOption("3PieceJKLPolesDriveOnly", AutoBuilder.buildAuto("3PieceJKLPolesOld"));
        autoChooser.addOption("DriveForwordAndIntakeTest", AutoBuilder.buildAuto("DriveForwordAndIntakeTest"));
        autoChooser.addOption("MiddleStart1Coral", AutoBuilder.buildAuto("MiddleStart1Coral"));
        SmartDashboard.putData("autos", autoChooser);
        //Shuffleboard.getTab("Test").add("test!!!",autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
