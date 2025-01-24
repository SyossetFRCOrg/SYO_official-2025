// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import java.util.function.Supplier;

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

    private final PIDController pidController;

    /** Creates a new ElevatorSubsystem. */
    public ElevatorSubsystem() {
        leftMotor = new SparkMax(16, MotorType.kBrushless);
        rightMotor = new SparkMax(17, MotorType.kBrushless);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        //configuring motors 
        var leftMotorConfig = new SparkMaxConfig();
        var rightMotorConfig = new SparkMaxConfig();
        
        leftMotorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40, 40)
            .voltageCompensation(12.0);

        rightMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40, 40)
            .voltageCompensation(12.0);

        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        //reset voltage to 0
        setVoltage(0.0);

        pidController = new PIDController(4.0, 0.0, 0.0);
    }

    public double getPosition() {
        //averages encoder positions and returns position
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
    }

    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -1.0, 2.0); //TO DO, CLAMP VALUES
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }
    
    public class Hover extends Command {
        private double target;

        public Hover() {
            addRequirements(ElevatorSubsystem.this);
        }

        @Override
        public void initialize() {
            this.target = getPosition();

            pidController.reset();
            pidController.setSetpoint(target);
        }

        @Override
        public void execute() {
            setVoltage(pidController.calculate(getPosition()));
        }
    }

    public class SetPosition extends Command {
        private double target;

        public SetPosition(double target) {
            this.target = target;
            addRequirements(ElevatorSubsystem.this);
        }

        @Override
        public void initialize() {
            pidController.reset();
            pidController.setSetpoint(target);
        }

        @Override
        public void execute() {
            setVoltage(pidController.calculate(getPosition()));
        }

        @Override
        public boolean isFinished() {
            return Math.abs(getPosition() - target) < 10;
        }
    }

    public class FreeMove extends Command {
        private final Supplier<Double> movementSupplier;

        public FreeMove(Supplier<Double> movementSupplier) {
            this.movementSupplier = movementSupplier;
        }

        @Override
        public void execute() {
            setVoltage(movementSupplier.get());
        }
    }
}
