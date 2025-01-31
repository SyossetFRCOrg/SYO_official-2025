package frc.robot.subsystems.drive.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Module extends SubsystemBase {
    public static class Config implements Cloneable {
        public String name = "Swerve Module";

        public double xPos;
        public double yPos;

        public double wheelRadius;
        public double maxSpeedRad = (5600.0 / 60.0) / ((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0));

        public Object clone() throws CloneNotSupportedException {
            return super.clone();
        }
    }

    private final ModuleIO io;
    private ModuleIO.Inputs inputs = new ModuleIO.Inputs();

    private final PIDController turnController;
    
    private final Translation2d pos;
    private final double wheelRadius;
    @SuppressWarnings("unused")
    private final double maxSpeed;

    private RunState runStateCommand;

    public Module(Config config, ModuleIO io) {
        setName(config.name);
        this.io = io;

        turnController = new PIDController(5.0, 0.0, 0.0);
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        wheelRadius = config.wheelRadius;
        maxSpeed = config.maxSpeedRad * wheelRadius * 2 * Math.PI;
        pos = new Translation2d(config.xPos, config.yPos);

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

    public Rotation2d getAngle() {
        return inputs.turnPosition;
    }

    public void setState(SwerveModuleState state) {
        runStateCommand.setState(state);
    }

    public Translation2d getPos() {
        return pos;
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
            turnController.reset();
            turnController.setSetpoint(state.angle.getRadians());
        }

        @Override
        public void execute() {
            io.setDriveVelocity(state.speedMetersPerSecond / wheelRadius);
            io.setTurnVelocity(state.angle.minus(inputs.turnPosition).getRadians());
            // io.setTurnPosition(state.angle.minus(inputs.turnZeroRotation).getRadians());
        }
    }
}
