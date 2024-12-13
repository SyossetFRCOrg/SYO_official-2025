// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class IntakeIOSparkMax implements IntakeIO {

  private static final double intakeGearRatio = (1.0 / 5.0);
  public static final double maxIntakeRate = 5676.0 * intakeGearRatio; // rpm

  private static final double rotateGearRatio = (1.0 / 100.0) * (60.0 / 64.0);
  public static final double maxRotateAngularSpeed = 5676.0 * rotateGearRatio * 2.0 * Math.PI / 60; // rad/s

  private final DigitalInput lowLimitSwitch;
  private final DigitalInput highLimitSwitch;


  private double m_intakeRate;
  private double m_angularSpeed;

  private final CANSparkMax  intake_motor = new CANSparkMax(24, MotorType.kBrushless);
  private final CANSparkMax  rotate_motor = new CANSparkMax(41, MotorType.kBrushless);

  private final RelativeEncoder intake_encoder = intake_motor.getEncoder();
  private final RelativeEncoder rotate_encoder = rotate_motor.getEncoder();
    
  ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
  ShuffleboardLayout intakeLayout = tab.getLayout("Intake", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);
  private final GenericEntry m_intakeRateEntry = intakeLayout.add("Intake Rate", 0 + " rpm").getEntry();
  private final GenericEntry m_rotateAngleEntry = intakeLayout.add("Intake Angle", 0 + " rad").getEntry();
  private final GenericEntry m_rotateAngularSpeedEntry = intakeLayout.add("Intake Angular Speed", 0 + " rad/s").getEntry();
  


  public IntakeIOSparkMax() {

    intake_encoder.setPositionConversionFactor(intakeGearRatio); // rot
    intake_encoder.setVelocityConversionFactor(intakeGearRatio); // rpm

    rotate_encoder.setPositionConversionFactor(rotateGearRatio * 2.0 * Math.PI); // rad
    rotate_encoder.setVelocityConversionFactor(rotateGearRatio * 2.0 * Math.PI / 60); // rad/s

    intake_motor.setIdleMode(IdleMode.kBrake);
    rotate_motor.setIdleMode(IdleMode.kBrake);

    intake_motor.setInverted(false);
    rotate_motor.setInverted(true);

    
    lowLimitSwitch = new DigitalInput(4);
    highLimitSwitch = new DigitalInput(6);

    intake_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 200);
    rotate_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 200);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    
    inputs.positionRad = (rotate_encoder.getPosition());


    inputs.velocityRadPerSec =(rotate_encoder.getVelocity());

    inputs.appliedVoltsIntake = intake_motor.getAppliedOutput() * intake_motor.getBusVoltage();

    inputs.appliedVoltsRotate = rotate_motor.getAppliedOutput() * rotate_motor.getBusVoltage();

    inputs.currentAmps = new double[] {intake_motor.getOutputCurrent(), rotate_motor.getOutputCurrent()};

  }

  /**
     * Engages the intake.
     * 
     * @param intakeRate The rate of intake (rpm).
     */
    public void intake(double intakeRate) {
      intake_motor.setVoltage(intakeRate);

  }

  /**
   * Changes the intake angle.
   * 
   * @param power The power desired
   */
  public void rotate(double power) {
      rotate_motor.setVoltage(power);
      
  }

  @Override
    public void stop(boolean interrupted) {
        intake_motor.stopMotor();
        rotate_motor.stopMotor();
      }

  /** Returns the current angle of the intake (rad). */
  public double getAngle() {
      return rotate_encoder.getPosition();
  }

  /** Resets the angle of the intake to 0. */
  public void reset() {
      rotate_encoder.setPosition(0);
  }

  /** Displays the periodically updated intake rate on the Shuffleboard */
  public void updateShuffleboard() {
      m_intakeRateEntry.setString(intake_encoder.getVelocity() + " rpm");
      m_rotateAngleEntry.setString(rotate_encoder.getPosition() + " rad");
      m_rotateAngularSpeedEntry.setString(rotate_encoder.getVelocity() + " rad/s");
      
  }



  /** Returns whether the intake can be activated. */
  public boolean canIntake() {
      return true;
  }

  /** Returns whether the intake can rotate. */
  public boolean canRotate() {
      if (m_angularSpeed > 0 && !highLimitSwitch.get()) {
          return false;
      }
      if (m_angularSpeed < 0 && !lowLimitSwitch.get()) {
          return false;
      }
      return true;
  }

  // @Override
  // public void configurePID(double kP, double kI, double kD) {
  //   pid.setP(kP);
  //   pid.setI(kI);
  //   pid.setD(kD);
  //   // pid.setFF(0);
  // }
}
