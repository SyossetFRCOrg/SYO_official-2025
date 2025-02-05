// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Elevator extends SubsystemBase {
    public static final double LOWER_LIMIT = 0.0;
    public static final double UPPER_LIMIT = 120.0;

    private final MotorIO motor;
    private final MotorIO.Inputs inputs = new MotorIO.Inputs();

    private final MotorCommands commands;

    /** Creates a new ElevatorSubsystem. */
    public Elevator() {
        MotorIOSpark.Config config = new MotorIOSpark.Config();

        config.canid = 16;
        config.motorType = MotorType.kBrushless;
        config.idleMode = IdleMode.kBrake;
        config.stallLimit = 60;
        config.freeLimit = 0;

        config.gearRatio = 4.0;
        config.minOutput = -0.8;
        config.maxOutput = 0.8;

        config.kP = 32.0;
        config.kI = 0.0;
        config.kD = 0.0;
        config.kF = 0.0;

        config.maxVelocity = 24.0;
        config.maxAcceleration = 96.0;

        config.feedForward = new ElevatorFeedForward(0.5, 0.1, 0.0, 0.75);

        motor = new MotorIOSpark(config);
        motor.resetPosition(0.0);
        SmartDashboard.putData("Elevator PID", ((MotorIOSpark)motor).getPid());
        SmartDashboard.putData("Elevator", this);

        commands = new MotorCommands(this, motor);
        setDefaultCommand(commands.new Hover());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Position", () -> inputs.positionRad, null);
        builder.addDoubleProperty("Velocity", () -> inputs.velocityRadPerSec, null);
        builder.addDoubleProperty("Voltage", () -> inputs.appliedVolts, null);
        builder.addDoubleProperty("Current", () -> inputs.currentAmps, null);
    }

    @Override
    public void periodic() {
        motor.updateInputs(inputs);
    }

    public void debugControls(CommandXboxController controller) {
        var ctrlMode = controller.rightTrigger().and(controller.leftTrigger().negate());
        ctrlMode.and(controller.x().or(controller.y()))
            .whileTrue(commands.new FreeMove(() -> (controller.y().getAsBoolean() ? 24.0 : 0.0) - (controller.x().getAsBoolean() ? 24.0 : 0.0)));
        ctrlMode.and(controller.povLeft()).onTrue(commands.new ResetPosition());
    }

    public Command getPositionCommand(double position) {
        return commands.new MoveToPosition(position);
    }

    public class ElevatorFeedForward implements FeedForward, Sendable {
        private double kS;
        private double kV;
        private double kA;
        private double kG;

        public ElevatorFeedForward(double kS, double kV, double kA, double kG) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.kG = kG;
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("kS", () -> kS, x -> kS = x);
            builder.addDoubleProperty("kV", () -> kV, x -> kV = x);
            builder.addDoubleProperty("kA", () -> kA, x -> kA = x);
            builder.addDoubleProperty("kG", () -> kG, x -> kG = x);
        }
        @Override
        public double calculate(double position, double velocity, double acceleration) {
            return kS * Math.signum(velocity) + kV * velocity + kA * acceleration + kG;   
        }
    }
}