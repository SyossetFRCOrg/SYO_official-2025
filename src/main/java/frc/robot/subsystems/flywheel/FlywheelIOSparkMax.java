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

package frc.robot.subsystems.flywheel;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class FlywheelIOSparkMax implements FlywheelIO {
  private static final double GEAR_RATIO = 1.0/3.0 ;    
  public static final double maxOuttakeRate = 5676.0 * GEAR_RATIO; // rpm


  private final CANSparkMax leader = new CANSparkMax(60, MotorType.kBrushless);
  private final CANSparkMax follower = new CANSparkMax(23, MotorType.kBrushless);
  private final CANSparkMax linear_actuator = new CANSparkMax(19, MotorType.kBrushed);
    



  private final DutyCycleEncoder rotate_encoder = new DutyCycleEncoder(7); //(actual number of) ->Constants.OUTTAKE_ROTATE_ENCODER);
  private final RelativeEncoder leader_encoder = leader.getEncoder();
  private final RelativeEncoder follower_encoder = follower.getEncoder();
  
  private final SparkPIDController pid = leader.getPIDController();




  ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
  ShuffleboardLayout outtakeLayout = tab.getLayout("Outtake", BuiltInLayouts.kList).withSize(2, 3).withPosition(2, 0);
  private final GenericEntry m_outtakeRateEntry = outtakeLayout.add("Outtake Flywheel 1 rpm", (leader_encoder.getVelocity()/GEAR_RATIO )).getEntry();
  private final GenericEntry m_outtakeRateEntry2 = outtakeLayout.add("Outtake Flywheel 2 rpm", (follower_encoder.getVelocity()/GEAR_RATIO)).getEntry();
  private final GenericEntry m_linearActuatorPositionEntry = outtakeLayout.add("Outtake Angle rad", getAngle()).getEntry();
  private final GenericEntry outtakeMotorsCurrentEntry = outtakeLayout.add("Average Current Drawn (Amps)", (getCurrentOutput())).getEntry();


  public FlywheelIOSparkMax() {
    leader.restoreFactoryDefaults();
    follower.restoreFactoryDefaults();
    linear_actuator.restoreFactoryDefaults();


    leader.setCANTimeout(250);
    follower.setCANTimeout(250);
    linear_actuator.setCANTimeout(250);
    

    leader.setInverted(false);
    follower.follow(leader, false);
    linear_actuator.setInverted(false);

    leader.setIdleMode(IdleMode.kCoast);
    follower.setIdleMode(IdleMode.kCoast);
    linear_actuator.setIdleMode(IdleMode.kBrake);



    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(30);
    leader.burnFlash();
    follower.burnFlash();

    leader.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 200);
    follower.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 200);



    
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {

    
    inputs.positionRad = Units.rotationsToRadians(leader_encoder.getPosition() * GEAR_RATIO);


    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond((leader_encoder.getVelocity() + follower_encoder.getVelocity()) / 2.0 * GEAR_RATIO);

    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();

    inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};

  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  /**
   * Returns the current angle of the outtake.
   * 
   * @return Angle of the outtake (rad).
   */
  public double getAngle() {
    return -2 * Math.PI * rotate_encoder.getAbsolutePosition();
}

/**
 * calculates
 * @return returns the average current drawn from the two flywheels
 */
public double getCurrentOutput() {
  return (leader.getOutputCurrent() + follower.getOutputCurrent()) / 2.0;
}

    /**
     * Engages the actuator to rotate the outtake.
     * 
     * @param actuatorPower Linear actuator power [-1.0, 1.0].
     */
  public void rotate (double actuatorPower){
    linear_actuator.set(actuatorPower);
  }



  /** Displays the periodically updated outtake rate on the Shuffleboard */
  public void updateShuffleboard() {
      m_outtakeRateEntry.setDouble((leader_encoder.getVelocity()) * GEAR_RATIO);
      m_outtakeRateEntry2.setDouble((follower_encoder.getVelocity())* GEAR_RATIO);
      m_linearActuatorPositionEntry.setDouble(getAngle());
      outtakeMotorsCurrentEntry.setDouble(getCurrentOutput());
  }

  // @Override
  // public void setVelocity(double velocityRadPerSec, double ffVolts) {
  //   pid.setReference(
  //       Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
  //       ControlType.kVelocity,
  //       0,
  //       ffVolts,
  //       ArbFFUnits.kVoltage);
  // }

    /**
     * Engages the outtake.
     * 
     * @param outtakeRate The rate of outtake (rpm).
     */
    public void setVelocity(double velocityRPM, double ffVolts) {
      setVoltage(ffVolts);
  }


  @Override
  public void stop() {
    leader.stopMotor();
    rotate(0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }

  @Override
  public double getMaxOuttakeRate(){
    return maxOuttakeRate;
  }
}
