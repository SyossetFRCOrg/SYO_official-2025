// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ArmConstants.*;

import java.util.function.Supplier;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax armMotor;
    private final RelativeEncoder armEncoder;
    private final SparkMaxConfig armMotorConfig;

    private final PIDController pidController;

    /** Creates a new ArmSubsystem. */
    public ArmSubsystem() {
        armMotor = new SparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
        armEncoder = armMotor.getEncoder();

        armMotorConfig = new SparkMaxConfig();

        armMotorConfig.inverted(true);
        armMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ARM_CURRENT_LIMIT);

        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        setArmVoltage(0.0);

        pidController = new PIDController(0.2, 0.0, 0.0);
        pidController.reset();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", armEncoder.getPosition());
    }

    public void setArmVoltage(double voltage) {
        armMotor.set(MathUtil.clamp(voltage, -2.0, 2.0));
    }

    public final Command runArmMotorCommand = Commands.startEnd(
            () -> setArmVoltage(ARM_VOLTAGE),
            () -> setArmVoltage(0),
            this).withName("arm.runArmMotor");

    public class Hover extends Command {
        private double target;

        public Hover() {
            addRequirements(ArmSubsystem.this);
        }

        @Override
        public void initialize() {
            this.target = armEncoder.getPosition();

            pidController.reset();
            pidController.setSetpoint(target);
        }

        @Override
        public void execute() {
            setArmVoltage(pidController.calculate(armEncoder.getPosition()));
        }
    }

    public class SetPosition extends Command {
        private double target;

        public SetPosition(double target) {
            this.target = target;
            addRequirements(ArmSubsystem.this);
        }

        @Override
        public void initialize() {
            pidController.reset();
            pidController.setSetpoint(target);
        }

        @Override
        public void execute() {
            setArmVoltage(pidController.calculate(armEncoder.getPosition()));
        }

        @Override
        public boolean isFinished() {
            return Math.abs(armEncoder.getPosition() - target) < 10;
        }
    }

    public class FreeMove extends Command {
        private final Supplier<Double> movementSupplier;

        public FreeMove(Supplier<Double> movementSupplier) {
            this.movementSupplier = movementSupplier;
            addRequirements(ArmSubsystem.this);
        }

        @Override
        public void execute() {
            setArmVoltage(movementSupplier.get());
        }

        @Override
        public void end(boolean interrupted) {
            setArmVoltage(0.0);
        }
    }
}
