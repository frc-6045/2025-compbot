// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class ArmSubsystem extends SubsystemBase {
/** thing */
  private final SparkFlex m_ArmMotor;
  private final AbsoluteEncoder m_AbsoluteEncoder;
  SparkFlexConfig config = new SparkFlexConfig();

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    m_ArmMotor = new SparkFlex(MotorConstants.kSparkFlexArmMotorCANID, MotorType.kBrushless);
    //m_AbsoluteEncoder = new DutyCycleEncoder(9);
    updateMotorSettingsForTest(m_ArmMotor);
    m_ArmMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_AbsoluteEncoder = m_ArmMotor.getAbsoluteEncoder();
  }

   public void updateMotorSettingsForTest(SparkFlex motor) {
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kIntakeMotorsCurrentLimit);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
  }

  public void setSpeed(double speed) {
    if (speed>MotorConstants.kSparkFlexArmMotorMaxSpeed || speed<-MotorConstants.kSparkFlexArmMotorMaxSpeed)
      speed = MotorConstants.kSparkFlexArmMotorMaxSpeed;
    m_ArmMotor.set(speed);
  }

  public void stopArmMotor() {
    m_ArmMotor.stopMotor();
  }

  public AbsoluteEncoder getAbsoluteEncoder() {
    return m_AbsoluteEncoder;
  }

  public double getAbsoluteEncoderPosition() {
    return m_AbsoluteEncoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm", getAbsoluteEncoderPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
