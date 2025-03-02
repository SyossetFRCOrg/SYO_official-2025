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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DeepHangConstants;

public class DeepHangSubsystem extends SubsystemBase {
  private final SparkMax deepHangMotor;
  private final RelativeEncoder deepHangEncoder;
  private final SparkMaxConfig deepHangMotorConfig;
  private double deepHangVoltage;
  private double deepHangBoostVoltage;

  // TODO
  public final SetPosition grab = new SetPosition(50);

  private final PIDController pidController;

  /** Creates a new DeepHangSubsystem. */
  public DeepHangSubsystem() {
    deepHangMotor = new SparkMax(DeepHangConstants.DEEP_HANG_MOTOR_ID, SparkMax.MotorType.kBrushless);
    deepHangEncoder = deepHangMotor.getEncoder();

    deepHangMotorConfig = new SparkMaxConfig();

    deepHangMotorConfig.inverted(true);

    deepHangMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(DeepHangConstants.DEEP_HANG_CURRENT_LIMIT);

    deepHangMotor.configure(deepHangMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    deepHangVoltage = DeepHangConstants.DEEP_HANG_VOLTAGE;

    deepHangBoostVoltage = 2.0;

    // TODO these values are placeholders and need to be tuned
    pidController = new PIDController(0.2, 0.0, 0.0);
    pidController.reset();

  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Deep Hang Set Voltage", () -> deepHangVoltage, (value) -> deepHangVoltage = value);
    builder.addDoubleProperty("Deep Hang Applied Voltage", () -> deepHangMotor.getAppliedOutput(), null);
  }

  public void setDeepHangVoltage(double voltage) {
    System.out.println(voltage);
    deepHangMotor.set(voltage);
  }

  public double getPosition() {
    return deepHangEncoder.getPosition();
  }

  public final Command runDeepHangForwardCommand = Commands.startEnd(
      () -> setDeepHangVoltage(deepHangVoltage),
      () -> setDeepHangVoltage(0),
      this).withName("deephang.runDeepHang");

  public final Command runDeepHangBackwardCommand = Commands.startEnd(
      () -> setDeepHangVoltage(-deepHangVoltage),
      () -> setDeepHangVoltage(0),
      this).withName("deephang.runDeepHang");

  public Command getRunDeepHangMotorBackwardCommand(boolean increaseVoltage) {
    return new InstantCommand(
        () -> setDeepHangVoltage(-deepHangVoltage - (increaseVoltage ? deepHangBoostVoltage : 0.0)));
  }
  public Command getRunDeepHangMotorForwardCommand(boolean increaseVoltage) {
    return new InstantCommand(
        () -> setDeepHangVoltage(deepHangVoltage + (increaseVoltage ? deepHangBoostVoltage : 0.0)));
  }

  public Command getStopDeepHangMotorCommand() {
    return new InstantCommand(() -> setDeepHangVoltage(0));
  }

  public class SetPosition extends Command {
    private double target;

    public SetPosition(double target) {
      this.target = target;
      addRequirements(DeepHangSubsystem.this);
    }

    @Override
    public void initialize() {
      pidController.reset();
      pidController.setSetpoint(target);
    }

    @Override
    public boolean isFinished() {
      return Math.abs(getPosition() - target) < 10;
    }
  }
}
