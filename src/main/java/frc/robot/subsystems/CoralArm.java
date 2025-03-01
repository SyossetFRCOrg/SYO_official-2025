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

public class CoralArm extends SubsystemBase {
    public static final double LOWER_LIMIT = -Math.PI / 2;
    public static final double UPPER_LIMIT = Math.PI / 2;

    private final MotorIO motor;
    private final MotorIO.Inputs inputs = new MotorIO.Inputs();

    private final MotorCommands commands;
    private final ArmFeedForward feedForward;

    /** Creates a new ArmSubsystem. */
    public CoralArm() {
        MotorIOSpark.Config config = new MotorIOSpark.Config();

        config.canid = 19;
        config.inverted = true;
        config.motorType = MotorType.kBrushless;
        config.idleMode = IdleMode.kBrake;
        config.stallLimit = 60;
        config.freeLimit = 0;

        config.gearRatio = 15.0;
        config.minOutput = -0.5;
        config.maxOutput = 0.5;

        config.kP = 4.0;
        config.kI = 0.0;
        config.kD = 0.0;
        
        config.vkP = 0.7;
        config.vkI = 0.0;
        config.vkD = 0.0;

        config.maxVelocity = 8.0;
        config.maxAcceleration = 16.0;
        config.maxJerk = 32.0;
        
        config.debounceTime = 0.2;
        config.tolerance = 0.1;

        feedForward = new ArmFeedForward(0.0, 0.25, 0.0, 0.35);
        config.feedForward = feedForward;

        motor = new MotorIOSpark(config);
        motor.resetPosition(-Math.PI/2);
        SmartDashboard.putData("Coral Arm", this);
        ((MotorIOSpark)motor).putData("Coral Arm");
        
        commands = new MotorCommands(this, motor);
        setDefaultCommand(commands.new Hover());
    }

    @Override
    public void periodic() {
        motor.updateInputs(inputs);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Position", () -> inputs.positionRad, null);
        builder.addDoubleProperty("Velocity", () -> inputs.velocityRadPerSec, null);
        builder.addDoubleProperty("Voltage", () -> inputs.appliedVolts, null);
        builder.addDoubleProperty("Current", () -> inputs.currentAmps, null);
    }

    public Command getPositionCommand(double position) {
        return commands.new MoveToPosition(position);
    }

    public Command getSetKg(double kG) {
        return Commands.runOnce(() -> feedForward.kG = kG);
    }

    public Command getResetPosition()
    {
        return Commands.runOnce(() -> motor.resetPosition(-Math.PI/2));
    }

    public void debugControls(CommandXboxController controller) {
        var ctrlMode = controller.rightTrigger().negate().and(controller.leftTrigger().negate());
        ctrlMode.whileTrue(commands.new FreeMove(() -> (controller.getRightY() > 0 ? -4.0 : (controller.getRightY() < 0 ? 4.0 : 0))));
        ctrlMode.and(controller.povRight()).onTrue(commands.new ResetPosition(-Math.PI/2));
    }

    public class ArmFeedForward implements FeedForward {
        private double kS;
        private double kV;
        private double kA;
        private double kG;

        public ArmFeedForward(double kS, double kV, double kA, double kG) {
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
            return Math.cos(position) * kG + Math.signum(velocity) * kS + velocity * kV + acceleration * kA;
        }
    }
}
