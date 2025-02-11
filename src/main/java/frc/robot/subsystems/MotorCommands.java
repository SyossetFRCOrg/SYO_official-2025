package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class MotorCommands {
    private final Subsystem subsystem;
    private final MotorIO motor;

    public MotorCommands(Subsystem subsystem, MotorIO motor) {
        this.subsystem = subsystem;
        this.motor = motor;
    }

    public class Hover extends Command {
        public Hover() {
            addRequirements(subsystem);
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
            addRequirements(subsystem);
            motor.setVelocity(velocity);
        }
    }
    
    public class FreeMove extends Command {
        public final Supplier<Double> velocitySupplier;

        public FreeMove(Supplier<Double> velocitySupplier) {
            this.velocitySupplier = velocitySupplier;
            addRequirements(subsystem);
        }

        @Override
        public void execute() {
            motor.setVelocity(velocitySupplier.get());
        }
    }

    public class ResetPosition extends Command {
        private final double position;

        public ResetPosition(double position) {
            this.position = position;
        }

        public ResetPosition() {
            position = 0.0;
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
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            motor.setSetpoint(position);
        }

        @Override
        public boolean isFinished() {
            return motor.atSetpoint();
        }
    }
}
