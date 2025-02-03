// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralArm extends SubsystemBase {
    private final MotorIO motor;
    private final MotorIO.Inputs inputs = new MotorIO.Inputs();

    /** Creates a new ArmSubsystem. */
    public CoralArm() {
        MotorIOSpark.Config config = new MotorIOSpark.Config();

        config.canid = 19;
        config.motorType = MotorType.kBrushless;
        config.idleMode = IdleMode.kBrake;
        config.stallLimit = 40;
        config.freeLimit = 0;

        config.gearRatio = 1.0;
        config.minOutput = -0.2;
        config.maxOutput = 0.2;

        config.kP = 2.0;
        config.kI = 0.0;
        config.kD = 0.0;

        config.kS = 0.0;
        config.kV = 0.02;
        config.kA = 0.04;
        config.kG = 2.26;

        config.maxVelocity = 1.0;
        config.maxAcceleration = 2.0;

        config.feedForward = FeedForwardType.ARM;

        motor = new MotorIOSpark(config);
        motor.setPosition(0.0);
        SmartDashboard.putData("Elevator Motor", (MotorIOSpark)motor);
        SmartDashboard.putData("Elevator", this);
    }

    @Override
    public void periodic() {
        motor.updateInputs(inputs);
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
