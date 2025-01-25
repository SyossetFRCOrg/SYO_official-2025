// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DeepHangConstants;

public class DeepHangSubsystem extends SubsystemBase {
  private final SparkMax deepHangMotor;
  private final RelativeEncoder deepHangEncoder;
  private final SparkMaxConfig deepHangMotorConfig;

  private final PIDController pidController;

  /** Creates a new DeepHangSubsystem. */
  public DeepHangSubsystem() {
    deepHangMotor = new SparkMax(DeepHangConstants.DEEP_HANG_MOTOR_ID, SparkMax.MotorType.kBrushless);
    deepHangEncoder = deepHangMotor.getEncoder();

    deepHangMotorConfig = new SparkMaxConfig();

    deepHangMotorConfig.inverted(true);

    deepHangMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(DeepHangConstants.DEEP_HANG_CURRENT_LIMIT);

    deepHangMotor.configure(deepHangMotorConfig,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    

    //TODO these values are placeholders and need to be tuned
    pidController = new PIDController(0.2, 0.0, 0.0);
    pidController.reset();
  
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Deep Hang Position", deepHangEncoder.getPosition());
  }

  public void setDeepHangVoltage(double voltage) {
    deepHangMotor.set(MathUtil.clamp(voltage, -3, 3));
  }

  public final Command runDeepHangMotor = Commands.startEnd(
    () -> setDeepHangVoltage(DeepHangConstants.DEEP_HANG_VOLTAGE),
    () -> setDeepHangVoltage(0.0)
  );

  
}
