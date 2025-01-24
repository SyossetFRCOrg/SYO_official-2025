package frc.robot.subsystems;

import java.io.File;
import org.syofrc.syolib.state.StateMachine;

import com.moandjiezana.toml.Toml;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.DefaultDriveCommand;
import frc.robot.subsystems.drive.Drivetrain;

public class Superstructure extends SubsystemBase {
    private final StateMachine<Superstructure> stateMachine = new StateMachine<>();
    private final CommandXboxController controller;

    private final Drivetrain drivetrain;
    private final AlgaeIntakeSubsystem algaeIntake;
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;

    public Superstructure() {
        var deployDir = Filesystem.getDeployDirectory();

        controller = new CommandXboxController(0);

        drivetrain = Drivetrain.create(new Toml().read(new File(deployDir, "config/swerve.toml")));
        drivetrain.setDefaultCommand(new DefaultDriveCommand(
        drivetrain, 
        () -> controller.getLeftY(), 
        () -> controller.getLeftX(), 
        () -> -controller.getRightX(), 
        5.0, 5.0
        ));

        algaeIntake = new AlgaeIntakeSubsystem();
        algaeIntake.setDefaultCommand(Commands.run(() -> algaeIntake.setRollerVoltage(0.0), algaeIntake));
        controller.leftBumper().whileTrue(Commands.run(() -> algaeIntake.setRollerVoltage(4.0), algaeIntake));
        controller.rightBumper().whileTrue(Commands.run(() -> algaeIntake.setRollerVoltage(-4.0), algaeIntake));

        elevator = new ElevatorSubsystem();
        elevator.setDefaultCommand(Commands.run(() -> elevator.setVoltage(0.0), elevator));
        controller.x().whileTrue(Commands.run(() -> elevator.setVoltage(2.0), elevator));
        controller.y().whileTrue(Commands.run(() -> elevator.setVoltage(-1.0), elevator));

        arm = new ArmSubsystem();
        arm.setDefaultCommand(Commands.run(() -> arm.setArmVoltage(0.0), arm));
        controller.a().whileTrue(Commands.run(() -> arm.setArmVoltage(0.5), arm));
        controller.b().whileTrue(Commands.run(() -> arm.setArmVoltage(-0.2), arm));
    }
}
