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
  private final SparkBase leftMotor;
  private final SparkBase rightMotor;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  // private final ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, MOVEMENT_CONSTRAINTS);
  // private final ElevatorFeedforward feedforwardController = new ElevatorFeedforward(kS, kG, kV, kA);
  
  private double target = 1000.0;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    // TODO figure out canids for motors
    leftMotor = new SparkMax(0, MotorType.kBrushless);
    rightMotor = new SparkMax(1, MotorType.kBrushless);

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    var leftMotorConfig = new SparkMaxConfig();
    var rightMotorConfig = new SparkMaxConfig();
    
    //TODO +- figure out which motor should be inverted
    leftMotorConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(20)
          .voltageCompensation(12.0);

    rightMotorConfig
          .inverted(true)
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(20)
          .voltageCompensation(12.0);

    //configuring motors
    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double pos = getPosition();

    //adjusts position until it reaches target. moves it up and down until really near the target
    if(Math.abs(pos - target) < 5) {
      setVoltage(0.0);
    } else if(pos < target) {
      setVoltage(3.0);
    } else if(pos >= target) {
      setVoltage(-3.0);
    }
  }

  public double getPosition() {
    return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
  }

  public void setTarget(double target) {
    this.target = target;
  }

  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -3.0, 3.0); //TO DO, CLAMP VALUES
    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(voltage);
  }
}
