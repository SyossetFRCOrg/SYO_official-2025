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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DeepHangConstants;

public class DeepHangSubsystem extends SubsystemBase {
  private final SparkMax deepHangMotor;
  private final RelativeEncoder deepHangEncoder;
  private final SparkMaxConfig deepHangMotorConfig;
  private double deepHangVoltage;

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

    // TODO these values are placeholders and need to be tuned
    pidController = new PIDController(0.2, 0.0, 0.0);
    pidController.reset();

  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Deep Hang Set Voltage", () -> deepHangVoltage, (value) -> deepHangVoltage = value);
    builder.addDoubleProperty("Deep Hang Applied Voltage", () -> deepHangMotor.getAppliedOutput(), null);
  }

  public void setDeepHangVoltage(double voltage) {
    deepHangMotor.set(MathUtil.clamp(voltage, -3, 3));
  }

  public double getPosition() {
    return deepHangEncoder.getPosition();
  }

  public Command getRunDeepHangMotorForwardCommand() {
    return new InstantCommand(() -> setDeepHangVoltage(deepHangVoltage));
  }

  public Command getRunDeepHangMotorBackwardCommand() {
    return new InstantCommand(() -> setDeepHangVoltage(-deepHangVoltage));
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
