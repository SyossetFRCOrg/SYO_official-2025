// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
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

        config.gearRatio = 1.0;
        config.minOutput = -0.8;
        config.maxOutput = 0.8;

        config.kP = 2.0;
        config.kI = 0.0;
        config.kD = 0.0;

        config.kS = 0.0;
        config.kV = 0.02;
        config.kA = 0.0;
        config.kG = 0.0;

        config.maxVelocity = 24.0;
        config.maxAcceleration = 48.0;

        config.feedForward = FeedForwardType.ELEVATOR;

        motor = new MotorIOSpark(config);
        motor.setPosition(0.0);
        SmartDashboard.putData("Elevator Motor", (MotorIOSpark)motor);
        SmartDashboard.putData("Elevator", this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Position", () -> inputs.positionRad, null);
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
        }

        @Override
        public void execute() {
            motor.setVelocity(velocity);
        }
    }

    public class ResetPosition extends Command {
        @Override
        public void initialize() {
            motor.setPosition(0.0);
        }
    }
}