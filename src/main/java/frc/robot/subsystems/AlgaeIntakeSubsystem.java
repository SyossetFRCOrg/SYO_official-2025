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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  
  private final SparkMax algaeIntakeRollerMotor;
  private SparkMaxConfig motorConfig;

  /** Creates a new AlgaeIntakeSubsystem. */
  public AlgaeIntakeSubsystem() {
    //Initializes Spark Max 
    algaeIntakeRollerMotor = new SparkMax(AlgaeIntakeConstants.ALGAE_INTAKE_ID, MotorType.kBrushless);
    
    /*
     * Create a new Spark Max configuration object. 
     * This stores the configuration parameters for the Spark Max to be set below
     */

    motorConfig = new SparkMaxConfig();
    motorConfig.inverted(true).idleMode(IdleMode.kCoast);
    motorConfig.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);
    algaeIntakeRollerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Sets the voltage of the Algae Intake Roller Motor
   * @param voltage the voltage to set the motor to
   */
  public void setRollerVoltage(double voltage) {
    algaeIntakeRollerMotor.set(voltage);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   * 
   */
  public boolean beamBroken() {
    //TODO: learn beam breaker code and implement it
    // Query some boolean state, such as a digital sensor.
    return false;
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
 
  /**
   * Command that runs the rollers in 
   * the direction to intake the Algae
   */
  public Command runIntakeRollersCommand() {
    return Commands.startEnd( 
        () -> setRollerVoltage(AlgaeIntakeConstants.ALGAE_INTAKE_SPEED), 
        () -> setRollerVoltage(0))
        .withName("intake.runIntakeRollers");
  }

  /**
   * Command that runs the rollers in 
   * the direction to outtake the Algae
   */
  public Command runOuttakeRollersCommand() {
    return Commands.startEnd( 
        () -> setRollerVoltage(-AlgaeIntakeConstants.ALGAE_INTAKE_SPEED), 
        () -> setRollerVoltage(0))
        .withName("intake.runOuttakeRollers");
      };

}
