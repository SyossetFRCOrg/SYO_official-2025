// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  SparkMax coralWrist;
  SparkMaxConfig motorConfig;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    coralWrist = new SparkMax(0,MotorType.kBrushless);

    //TODO Go through this and change it, just pasted in for syntax
    motorConfig = new SparkMaxConfig();
    motorConfig.inverted(true).idleMode(IdleMode.kBrake);
    motorConfig.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);
    coralWrist.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
