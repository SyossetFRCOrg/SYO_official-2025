package frc.robot.subsystems;

import java.io.File;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

import com.moandjiezana.toml.Toml;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.swerve.Drivetrain;
import frc.robot.subsystems.drive.swerve.ModuleIOSpark.Config.Drive;

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
                drive.setDefaultCommand(drive.joystickDrive(
                    drive,
                    () ->  0.5 * controller.getLeftY(),
                    () ->  0.5 * controller.getLeftX(),
                    () ->  0.4 * controller.getRightX()
                ));
            });

            controller.button(8).onTrue(drivetrain.get().setRotation(
                    DriverStation.getAlliance().get() == Alliance.Red ? (180) : (0)
            ));
        } else {
            drivetrain = Optional.empty();
        }


        if (subsystems.get("algae_intake").getBoolean("enabled")) {
            algaeIntake = Optional.of(new AlgaeIntakeSubsystem());
            algaeIntake.ifPresent(intake -> {
                intake.setDefaultCommand(Commands.run(() -> intake.setRollerVoltage(0.0), intake));
                controller.povUp().whileTrue(Commands.run(() -> intake.setRollerVoltage(2.0), intake));
                controller.povDown().whileTrue(Commands.run(() -> intake.setRollerVoltage(-2.0), intake));
            });
        } else {
            algaeIntake = Optional.empty();
        }

        if (subsystems.get("elevator_structure").getBoolean("enabled")) {
            elevatorStructure = Optional.of(new ElevatorStructure());
            elevatorStructure.ifPresent(structure -> structure.debugControls(controller));
        } else {
            elevatorStructure = Optional.empty();
        }
    }
}
