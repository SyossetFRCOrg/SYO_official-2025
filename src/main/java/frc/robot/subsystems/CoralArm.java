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

public class CoralArm extends SubsystemBase {
    public static final double LOWER_LIMIT = -Math.PI / 2;
    public static final double UPPER_LIMIT = Math.PI / 2;

    private final MotorIO motor;
    private final MotorIO.Inputs inputs = new MotorIO.Inputs();

    /** Creates a new ArmSubsystem. */
    public CoralArm() {
        MotorIOSpark.Config config = new MotorIOSpark.Config();

        config.canid = 19;
        config.inverted = true;
        config.motorType = MotorType.kBrushless;
        config.idleMode = IdleMode.kBrake;
        config.stallLimit = 40;
        config.freeLimit = 0;

        config.gearRatio = 15.0; // TODO
        config.minOutput = -0.5;
        config.maxOutput = 0.5;

        config.kP = 0.5;
        config.kI = 0.0;
        config.kD = 0.0;

        config.kS = 0.05;
        config.kV = 0.35;
        config.kA = 0.0;
        config.kG = 0.03; // TODO

        config.maxVelocity = 8.0;
        config.maxAcceleration = 8.0;

        config.feedForward = FeedForwardType.ARM;

        motor = new MotorIOSpark(config);
        motor.resetPosition(0.0);
        SmartDashboard.putData("Coral Arm Motor", (MotorIOSpark)motor);
        SmartDashboard.putData("Coral Arm", this);
    }

    @Override
    public void periodic() {
        motor.updateInputs(inputs);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Position", () -> inputs.positionRad, null);
        builder.addDoubleProperty("Velocity", () -> inputs.velocityRadPerSec, null);
    }

    public class Hover extends Command {
        public Hover() {
            addRequirements(CoralArm.this);
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
            addRequirements(CoralArm.this);
            motor.setVelocity(velocity);
        }

        @Override
        public void execute() {
            motor.setVelocity(velocity);
        }
    }
    
    public class FreeMove extends Command {
        public final Supplier<Double> velocitySupplier;

        public FreeMove(Supplier<Double> velocitySupplier) {
            this.velocitySupplier = velocitySupplier;
            addRequirements(CoralArm.this);
        }

        @Override
        public void execute() {
            motor.setVelocity(velocitySupplier.get());
            // motor.setVelocity(MathUtil.clamp(velocitySupplier.get(), LOWER_LIMIT - inputs.positionRad, UPPER_LIMIT - inputs.positionRad));
        }
    }

    public class ResetPosition extends Command {
        private final double position;

        public ResetPosition() {
            position = 0.0;
        }

        public ResetPosition(double position) {
            this.position = position;
        }

        @Override
        public void initialize() {
            motor.resetPosition(position);
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
            addRequirements(CoralArm.this);
        }

        @Override
        public void initialize() {
            motor.setSetpoint(position);
        }
    }
}
