// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

        config.kP = 4.0;
        config.kI = 0.0;
        config.kD = 0.0;

        config.vkP = 0.7;
        config.vkI = 0.0;
        config.vkD = 0.0;

        config.maxVelocity = 16.0;
        config.maxAcceleration = 64.0;
        config.maxJerk = 256.0;

        config.feedForward = new ElevatorFeedForward(0.0, 0.125, 0.0, 0.4);
        
        config.debounceTime = 0.2;
        config.tolerance = 0.2;

        motor = new MotorIOSpark(config);
        motor.resetPosition(0.0);
        SmartDashboard.putData("Elevator", this);
        ((MotorIOSpark)motor).putData("Elevator");

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

    public void debugControls(CommandXboxController subsystemController) {
        var ctrlMode = subsystemController.rightTrigger().negate().and(subsystemController.leftTrigger().negate());
        ctrlMode.and(subsystemController.povDown().or(subsystemController.povUp())).whileTrue(commands.new FreeMove(() -> (subsystemController.povUp().getAsBoolean() ? 16.0 : 0.0) - (subsystemController.povDown().getAsBoolean() ? 16.0 : 0.0)));
        ctrlMode.and(subsystemController.povLeft()).onTrue(commands.new ResetPosition());
    }

    public Command getPositionCommand(double position) {
        return commands.new MoveToPosition(position);
    }

    public Command getResetPosition(){
        return Commands.runOnce(() -> motor.resetPosition(0));
    }


    public class ElevatorFeedForward implements FeedForward {
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