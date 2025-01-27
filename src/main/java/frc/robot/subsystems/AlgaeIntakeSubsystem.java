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
        motorConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);
        algaeIntakeRollerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the voltage of the Algae Intake Roller Motor
     * @param voltage the voltage to set the motor to
     */
    public void setRollerVoltage(double voltage) {
        algaeIntakeRollerMotor.setVoltage(MathUtil.clamp(voltage, -4.0, 4.0));
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
     * Command that runs the rollers in 
     * the direction to intake the Algae
     */
    public final Command runIntakeRollersCommand = Commands.startEnd( 
            () -> setRollerVoltage(AlgaeIntakeConstants.ALGAE_INTAKE_SPEED), 
            () -> setRollerVoltage(0),
            this
    ).withName("intake.runIntakeRollers");

    /**
     * Command that runs the rollers in 
     * the direction to outtake the Algae
     */
    public final Command runOuttakeRollersCommand = Commands.startEnd( 
            () -> setRollerVoltage(AlgaeIntakeConstants.ALGAE_OUTTAKE_SPEED), 
            () -> setRollerVoltage(0),
            this
    ).withName("intake.runOuttakeRollers");

    public class RunAlgaeRollers extends Command {
        public boolean isRollingIn;

        public RunAlgaeRollers(boolean isRollingIn) {
            this.isRollingIn = isRollingIn;
            addRequirements(AlgaeIntakeSubsystem.this);
        }

        @Override
        public void initialize() {
            if (isRollingIn) {
                setRollerVoltage(AlgaeIntakeConstants.ALGAE_INTAKE_SPEED);
            } else {
                setRollerVoltage(AlgaeIntakeConstants.ALGAE_OUTTAKE_SPEED);
            }
        }

        @Override
        public void end(boolean interrupted) {
            setRollerVoltage(0);
        }
    }

}
