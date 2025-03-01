package frc.robot.subsystems;

import java.io.File;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

import com.moandjiezana.toml.Toml;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drive.swerve.Drivetrain;

public class Superstructure extends SubsystemBase {
    private final CommandXboxController driverController;
    private final CommandXboxController subsystemController;

    private final Optional<Drivetrain> drivetrain;
    private final Optional<AlgaeIntakeSubsystem> algaeIntake;
    private final Optional<ElevatorStructure> elevatorStructure;

    public Superstructure() {
        var deployDir = Filesystem.getDeployDirectory();
        var configDir = new File(deployDir, "config");

        Toml config = new Toml().read(new File(configDir, "superstructure.toml"));
        Map<String, Toml> subsystems = config.getTables("subsystems").stream()
                .collect(Collectors.toMap(toml -> toml.getString("id"), toml -> toml));

        driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
        subsystemController = new CommandXboxController(OperatorConstants.kSubsystemControllerPort);

        if (subsystems.get("drivetrain").getBoolean("enabled")) {
            drivetrain = Optional.of(new Drivetrain());
            // drivetrain = Optional.of(new Drivetrain(new Toml().read(new File(configDir,
            // "swerve.toml"))));
            drivetrain.ifPresent(drive -> {
                drive.setDefaultCommand(drive.joystickDrive(
                        drive,
                        () -> 0.6 * driverController.getLeftY(),
                        () -> 0.6 * driverController.getLeftX(),
                        () -> 0.5 * driverController.getRightX()));
            });

            driverController.button(8).onTrue(drivetrain.get().setRotation(
                    DriverStation.getAlliance().get() == Alliance.Red ? (180) : (0)));
        } else {
            drivetrain = Optional.empty();
        }

        if (subsystems.get("algae_intake").getBoolean("enabled")) {
            algaeIntake = Optional.of(new AlgaeIntakeSubsystem());
            algaeIntake.ifPresent(intake -> {
                intake.setDefaultCommand(
                        new InstantCommand(() -> intake.setRollerVoltage((subsystemController.getLeftY() < -0.1 ? -AlgaeIntakeConstants.ALGAE_INTAKE_SPEED : (subsystemController.getLeftY() > 0.1 ? AlgaeIntakeConstants.ALGAE_INTAKE_SPEED : 0.0))),algaeIntake.get()));
            });
        } else {
            algaeIntake = Optional.empty();
        }

        if (subsystems.get("elevator_structure").getBoolean("enabled")) {
            elevatorStructure = Optional.of(new ElevatorStructure(subsystemController));
            elevatorStructure.ifPresent(structure -> structure.debugControls());
        } else {
            elevatorStructure = Optional.empty();
        }
    }
}
