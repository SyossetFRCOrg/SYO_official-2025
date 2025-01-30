package frc.robot.subsystems.drive.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Module extends SubsystemBase {
    public static class Config implements Cloneable {
        public String name = "Swerve Module";

        public double xPos;
        public double yPos;

        public double wheelRadius;

        public Object clone() throws CloneNotSupportedException {
            return super.clone();
        }
    }

    private final ModuleIO io;
    private ModuleIO.Inputs inputs;

    private final PIDController turnController;
    
    private final double wheelRadius;

    private RunState runStateCommand;

    public Module(Config config, ModuleIO io) {
        setName(config.name);
        this.io = io;

        turnController = new PIDController(5.0, 0.0, 0.0);
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        this.wheelRadius = config.wheelRadius;

        runStateCommand = new RunState();
        setDefaultCommand(runStateCommand);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public ModuleIO.Inputs getInputs() {
        return inputs;
    }

    public void setState(SwerveModuleState state) {
        runStateCommand.setState(state);
    }

    private class RunState extends Command {
        private SwerveModuleState state;

        private RunState() {
            this.state = new SwerveModuleState();
            addRequirements(Module.this);
        }

        private void setState(SwerveModuleState state) {
            state.optimize(inputs.turnPosition);
            this.state = state;
            turnController.setSetpoint(state.angle.getRadians());
        }

        @Override
        public void execute() {
            io.setDriveVelocity(state.speedMetersPerSecond / wheelRadius);
            io.setTurnVelocity(turnController.calculate(inputs.turnPosition.getRadians()));
        }
    }
}
