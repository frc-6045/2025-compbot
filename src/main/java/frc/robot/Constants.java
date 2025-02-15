// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class MotorConstants {
    // arm constants
    public static final int kSparkFlexArmMotorCANID = 9;

    public static final int kSparkFlexArmMotorCurrentLimit = 40;
    public static final double kSparkFlexArmMotorSpeed = .2;
    public static final double kSparkFlexArmMotorMaxSpeed = 0.45;

    // elevator constants
    public static final int kSparkFlexElevatorMotor1CANID = 10;
    public static final int kSparkFlexElevatorMotor2CANID = 11;

    public static final int kSparkFlexElevatorMotorsCurrentLimit = 40;
    public static final double kSparkFlexElevatorMotorsSpeed = .65;
    public static final double kSparkFlexElevatorMotorsMaxSpeed = 0.9;

    // intake constants
    public static final int kIntakeMotor1CANID = 12;
    public static final int kIntakeMotor2CANID = 13;

    public static final int kIntakeMotorsCurrentLimit = 40;
    public static final double kIntakeMotorsSpeed = .5;
    public static final double kIntakeMotorsMaxSpeed = .6;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 1;
    public static final int kOperatorControllerPort = 0;
    public static final int kGodControllerPort = 2;
    public static double kHomeArmPosition;

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class PositionConstants {

    // A- Human player/coral intake setpoint
    public static final double kHumanArmPosition = 0.4262903034687042;
    public static final double kHumanElevatorPosition = 59;

    // Y- Home setpoint
    public static final double kHomeArmPosition = 0.32972583174705505;
    public static final double kHomeElevatorPosition = 14;

    // B- L1
    public static final double kL1ArmPosition = 0.46141165494918823;
    public static final double kL1ElevatorPosition = 0;

    // L2
    public static final double kL2ArmPosition = 0.277045;
    public static final double kL2ElevatorPosition = 22.1052;

    // left stick (top left paddle)- L3
    public static final double kL3ArmPosition = 0.9638405442237854;
    public static final double kL3ElevatorPosition = 0;

    // right stick (top right paddle)- L4
    public static final double kL4ArmPosition = 0.9521994590759277;
    public static final double kL4ElevatorPosition = 89.22;

    // arm flick goes to initialposition+kArmFlickDistance1 then to initialposition+kArmFlickDistance2
    public static final double kArmFlickDistance1=0.05;
    public static final double kArmFlickDistance2=-0.01;
  }

  public static class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    public static final double kFreeSpeedRpm = 6784; //NEO Motor 

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

        //TODO: maybe remove this multiplier
    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI ) 
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    //Drivetrain P
    public static final double kDrivingP = 0.01; //was 0.8
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps; 
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = .9; //rip spooky 0
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = -0.00; //FIXME: changed feed forward from 0
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // 80 amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static class AutoConstants {

  }

  public static class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    
    //Slew Constants
    public static final double kMaxSpeedMetersPerSecond =  5.74; //changed from 4.8 bc stuff wasnt straped down
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
    kMaxAngularSpeed; // was 4
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4; //was 2


    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(28); //22.5     24.9375  //TODO: find all of this
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(28); // 26.7  24.9375
    // Distance between front and back wheels on robot
    public static final double radiusMeters = Units.inchesToMeters(19.798); ///.8879 meters preconverted before
    //the distance from the center of the robot to the furthest swerve module

    public static final double kGyroOffsetX = Units.inchesToMeters(-3);
    public static final double kGyroOffsetY = Units.inchesToMeters(-2);
    
    //front left module
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kFrontLeftTurningCanId = 2;
    //front right module
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kFrontRightTurningCanId = 4;
    //rear left module
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kRearLeftTurningCanId = 6;
    //rear right module
    public static final int kRearRightDrivingCanId = 7;
    public static final int kRearRightTurningCanId = 8;

    // offsets
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kRearLeftChassisAngularOffset = Math.PI;
    public static final double kRearRightChassisAngularOffset = Math.PI / 2;


    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final boolean kGyroReversed = false;
        //change
        public static final double kTurningAngleP = 0;
        public static final double kTurningAngleI = 0;
        public static final double kTurningAngleD = 0;
  }
  
}

// maek controler wurk plez