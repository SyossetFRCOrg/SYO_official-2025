package frc.robot.subsystems;

import java.io.File;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

import com.moandjiezana.toml.Toml;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.swerve.Drivetrain;

public class Superstructure extends SubsystemBase {
    private final CommandXboxController controller;

    private final Optional<Drivetrain> drivetrain;
    private final Optional<AlgaeIntakeSubsystem> algaeIntake;
    private final Optional<ElevatorStructure> elevatorStructure;

    public Superstructure() {
        var deployDir = Filesystem.getDeployDirectory();
        var configDir = new File(deployDir, "config");

        Toml config = new Toml().read(new File(configDir, "superstructure.toml"));
        Map<String, Toml> subsystems = config.getTables("subsystems").stream().collect(Collectors.toMap(toml -> toml.getString("id"), toml -> toml));

        controller = new CommandXboxController(0);

        if (subsystems.get("drivetrain").getBoolean("enabled")) {
            drivetrain = Optional.of(new Drivetrain());
            // drivetrain = Optional.of(new Drivetrain(new Toml().read(new File(configDir, "swerve.toml"))));
            drivetrain.ifPresent(drive -> {
                drive.setDefaultCommand(drive.new DefaultDrive(
                    () -> 0.3 * controller.getLeftY(),
                    () -> 0.3 * controller.getLeftX(),
                    () -> 0.1 * controller.getRightX()
                ));
            });
        } else {
            drivetrain = Optional.empty();
        }

        if (subsystems.get("algae_intake").getBoolean("enabled")) {
            algaeIntake = Optional.of(new AlgaeIntakeSubsystem());
            algaeIntake.ifPresent(intake -> {
                intake.setDefaultCommand(Commands.run(() -> intake.setRollerVoltage(0.0), intake));
                controller.leftBumper().whileTrue(Commands.run(() -> intake.setRollerVoltage(4.0), intake));
                controller.rightBumper().whileTrue(Commands.run(() -> intake.setRollerVoltage(-4.0), intake));
            });
        } else {
            algaeIntake = Optional.empty();
        }

        if (subsystems.get("elevator_structure").getBoolean("enabled")) {
            elevatorStructure = Optional.of(new ElevatorStructure());
            elevatorStructure.ifPresent(structure -> {
                controller.x().or(controller.y())
                    .whileTrue(structure.getMoveElevator(() ->
                        (controller.y().getAsBoolean() ? 2.0 : 0.0) +
                        (controller.x().getAsBoolean() ? -0.8 : 0.0)
                    ));
                 
                controller.a().or(controller.b())
                    .whileTrue(structure.getMoveArm(() ->
                        (controller.b().getAsBoolean() ? 0.1 : 0.0) +
                        (controller.a().getAsBoolean() ? -0.1 : 0.0)
                        // (controller.b().getAsBoolean() ? 0.25 * (controller.rightTrigger().getAsBoolean() ? 8.0 : 1.0): 0.0) +
                        // (controller.a().getAsBoolean() ? -0.125 : 0.0)
                    ));
            });
        } else {
            elevatorStructure = Optional.empty();
        }
    }
}
