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

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    leftMotor = new SparkMax(0, MotorType.kBrushless);
    rightMotor = new SparkMax(1, MotorType.kBrushless);

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    var leftMotorConfig = new SparkMaxConfig();
    var rightMotorConfig = new SparkMaxConfig();

    leftMotorConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(20)
          .voltageCompensation(12.0);

    rightMotorConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(20)
          .voltageCompensation(12.0);

    //configuring motors
    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


 

  }

  // private double feedbackVoltage = 0;
  // private double feedForwardVoltage = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    var pos = getPosition();
    var target = 1000;

    //adjusts position until it reaches target. moves it up and down until really near the target
    if(Math.abs(pos - target) < 5)
    {
      leftMotor.set(0);
      rightMotor.set(0);
    }
    else if(pos < target) {
      leftMotor.set(0.25);
      rightMotor.set(-0.25); //FIX +- to see which one goes up or down
    } else if(pos >= target) {
      leftMotor.set(-0.25);
      rightMotor.set(0.25);
    }
    
  }

  public double getPosition() {
    return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
  }

  public double getVelocity() {
    return (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2;
  }


  private boolean initialized = false;

  public boolean getInitialized() {
    return initialized;
  }

  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -2.0, 2.0); //TO DO, CLAMP VALUES
    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(-voltage); //FIX +-
  }


}
