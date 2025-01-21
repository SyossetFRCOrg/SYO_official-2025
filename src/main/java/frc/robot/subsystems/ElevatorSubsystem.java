// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorSubsystem extends SubsystemBase {

  private final SparkBase motor;
  private final RelativeEncoder encoder;
  // private final ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, MOVEMENT_CONSTRAINTS);
  // private final ElevatorFeedforward feedforwardController = new ElevatorFeedforward(kS, kG, kV, kA);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    motor = new SparkMax(0, MotorType.kBrushless);
    encoder = motor.getEncoder();
    var motorConfig = new SparkMaxConfig();

    motorConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(20)
          .voltageCompensation(12.0);

    //configuring motor
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


    // motor = SparkConfigurator.createSparkMax(MOTOR_ID, MotorType.kBrushless, MOTOR_INVERTED,
    //         (s) -> s.setIdleMode(IdleMode.kBrake),
    //         (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
    //         (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS),
    //         (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0));


  }


  private double feedbackVoltage = 0;
  private double feedForwardVoltage = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    var pos = encoder.getPosition();
    var target = 1000;

    //adjusts position until it reaches target. moves it up and down until really near the target
    if(Math.abs(pos - target) < 5)
    {
      motor.set(0);
    }
    else if(pos < target) {
      motor.set(0.25);
    } else if(pos >= target) {
      motor.set(-0.25);
    }
    
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }


  private boolean initialized = false;

  public boolean getInitialized() {
    return initialized;
  }

  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -2.0, 2.0); //TO DO, CLAMP VALUES
    motor.setVoltage(voltage);
  }


}
