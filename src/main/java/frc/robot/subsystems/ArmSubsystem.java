// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ArmConstants.*;

public class ArmSubsystem extends SubsystemBase {
  private final SparkMax armMotor;
  private final RelativeEncoder armEncoder;
  private final SparkMaxConfig armMotorConfig;

  // private final ProfiledPIDController armPIDController;
  // private final ArmFeedforward feedforwardController;
  // private double feedbackVoltage = 0;


  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
      armMotor = new SparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
      armEncoder = armMotor.getEncoder();

      armMotorConfig = new SparkMaxConfig();

      armMotorConfig.inverted(true);
      armMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ARM_CURRENT_LIMIT);

      armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setArmVoltage(double voltage) {
    armMotor.set(voltage);
  }
  public Command runArmMotorCommand() {
    return Commands.startEnd( 
        () -> setArmVoltage(ARM_VOLTAGE), 
        () -> setArmVoltage(0))
        .withName("arm.runArmMotor");
  }


}
