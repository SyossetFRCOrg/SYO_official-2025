// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    public static final double LOWER_LIMIT = 0.0;
    public static final double UPPER_LIMIT = 120.0;

    private final MotorIO motor;
    private final MotorIO.Inputs inputs = new MotorIO.Inputs();

    /** Creates a new ElevatorSubsystem. */
    public Elevator() {
        MotorIOSpark.Config config = new MotorIOSpark.Config();

        config.canid = 16;
        config.motorType = MotorType.kBrushless;
        config.idleMode = IdleMode.kBrake;
        config.stallLimit = 40;
        config.freeLimit = 0;

        config.gearRatio = 4.0;
        config.minOutput = -0.8;
        config.maxOutput = 0.8;

        config.kP = 0.16;
        config.kI = 0.0;
        config.kD = 0.2;

        config.kS = 0.1;
        config.kV = 0.18;
        config.kA = 0.0;
        config.kG = 0.0; // TODO

        config.maxVelocity = 12.0;
        config.maxAcceleration = 24.0;

        config.feedForward = FeedForwardType.ELEVATOR;

        motor = new MotorIOSpark(config);
        motor.resetPosition(0.0);
        SmartDashboard.putData("Elevator Motor", (MotorIOSpark)motor);
        SmartDashboard.putData("Elevator", this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Position", () -> inputs.positionRad, null);
        builder.addDoubleProperty("Velocity", () -> inputs.velocityRadPerSec, null);
    }

    @Override
    public void periodic() {
        motor.updateInputs(inputs);
    }

    public class Hover extends Command {
        public Hover() {
            addRequirements(Elevator.this);
        }

        @Override
        public void execute() {
            motor.setVelocity(0.0);
        }
    }

    public class SetVelocity extends Command {
        public final double velocity;

        public SetVelocity(double velocity) {
            this.velocity = velocity;
            addRequirements(Elevator.this);
            motor.setVelocity(velocity);
        }

        @Override
        public void execute() {
            
        }
    }
    
    public class FreeMove extends Command {
        public final Supplier<Double> velocitySupplier;

        public FreeMove(Supplier<Double> velocitySupplier) {
            this.velocitySupplier = velocitySupplier;
            addRequirements(Elevator.this);
        }

        @Override
        public void execute() {
            motor.setVelocity(velocitySupplier.get());
            // motor.setVelocity(MathUtil.clamp(velocitySupplier.get(), LOWER_LIMIT - inputs.positionRad, UPPER_LIMIT - inputs.positionRad));
        }
    }

    public class ResetPosition extends Command {
        @Override
        public void initialize() {
            motor.resetPosition(0.0);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public class MoveToPosition extends Command {
        public final double position;

        public MoveToPosition(double position) {
            this.position = position;
            addRequirements(Elevator.this);
        }

        @Override
        public void initialize() {
            motor.setSetpoint(position);
        }
    }
}