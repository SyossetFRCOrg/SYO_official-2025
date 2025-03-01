// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
        motorConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);
        algaeIntakeRollerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the voltage of the Algae Intake Roller Motor
     * @param controllerInput value of the left stick y axis
     */
    public void setRollerVoltage(double controllerInput) {
        algaeIntakeRollerMotor.setVoltage(controllerInput > 0 ? -AlgaeIntakeConstants.ALGAE_INTAKE_SPEED : (controllerInput < 0 ? AlgaeIntakeConstants.ALGAE_INTAKE_SPEED : 0.0));
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     * 
     */
    public boolean beamBroken() {
        // TODO: learn beam breaker code and implement it
        // Query some boolean state, such as a digital sensor.
        return false;
    }
    
    /**
     * Command that runs the rollers in 
     * the direction to intake the Algae
     */
    public Command getRunIntakeRollersCommand(double controls){
        return new InstantCommand(() -> setRollerVoltage((controls < 0 ? -4.0 : (controls > 0 ? 4.0 : 0.0))));
    }


}
