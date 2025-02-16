package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Autos {
    private final SwerveSubsystem m_DriveSubsystem;
    private final IntakeSubsystem m_IntakeSubsystem;
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private final ArmSubsystem m_ArmSubsystem;
    private SendableChooser<Command> autoChooser;
    //private ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    public Autos(SwerveSubsystem drive, IntakeSubsystem intake, ElevatorSubsystem elev, ArmSubsystem arm) {
        m_IntakeSubsystem = intake;
        m_ElevatorSubsystem = elev;
        m_ArmSubsystem = arm;
        m_DriveSubsystem = drive;

        //incorprate pathplanner into code
        autoChooser = new SendableChooser<Command>();
        autoChooser.addOption("DoNothing", new InstantCommand(()->{System.out.println("hello I am doing nothing!");}));
        autoChooser.addOption("3piece", AutoBuilder.buildAuto("3PieceJKLTesting"));
        autoChooser.addOption("BackStart2Coral", AutoBuilder.buildAuto("BackStart2Coral"));
        //autoChooser.addOption("runIntake", new IntakeAuto(intake, 1, true));
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
