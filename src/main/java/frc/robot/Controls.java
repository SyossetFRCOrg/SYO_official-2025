package frc.robot;

import com.techhounds.houndutil.houndlib.subsystems.BaseDifferentialDrive.DifferentialDriveMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDState;

public class Controls {
    public static void configureControls(int port, Drivetrain drivetrain, Elevator elevator, Arm arm, Intake intake,
            Climber climber, CoralSim coralSim) {

        CommandXboxController controller = new CommandXboxController(port);

        drivetrain.setDefaultCommand(
                drivetrain.teleopDriveCommand(
                        () -> -controller.getLeftY(),
                        () -> -controller.getRightY(),
                        () -> -controller.getRightX(),
                        () -> controller.getHID().getRightStickButton(),
                        () -> DifferentialDriveMode.ARCADE));

        controller.a().whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L1, elevator, arm, coralSim));
        controller.x().whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L2, elevator, arm, coralSim));
        controller.b().whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L3, elevator, arm, coralSim));
        controller.y().whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L4, elevator, arm, coralSim));
        controller.start().whileTrue(RobotCommands.scoreCoralCommand(drivetrain, elevator, arm, coralSim));

        controller.povUp().whileTrue(RobotCommands.prepareIntakeCoralCommand(elevator, arm, coralSim));
        controller.povDown().whileTrue(RobotCommands.intakeCoralCommand(elevator, arm, coralSim));

        controller.povLeft().whileTrue(RobotCommands.prepareAlgaeL2RemoveCommand(elevator, arm));
        controller.povRight().whileTrue(RobotCommands.prepareAlgaeL3RemoveCommand(elevator, arm));
        controller.leftStick().whileTrue(RobotCommands.algaeRemoveCommand(drivetrain, elevator, arm));

        controller.rightBumper().whileTrue(intake.runRollersCommand());
        controller.leftBumper().whileTrue(intake.reverseRollersCommand());

        climber.setDefaultCommand(Commands
                .run(() -> climber.setVoltage(MathUtil
                        .applyDeadband((controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) * 4,
                                0.1)),
                        climber));

        controller.povRight().onTrue(GlobalStates.INITIALIZED.enableCommand());

        controller.back().toggleOnTrue(
                Commands.parallel(
                        elevator.setOverridenSpeedCommand(() -> -controller.getLeftY() * 0.25),
                        arm.setOverridenSpeedCommand(() -> -controller.getRightY() * 0.25),
                        Commands.run(drivetrain::stop, drivetrain))

                        .finallyDo(() -> {
                            elevator.resetControllersCommand().schedule();
                            arm.resetControllersCommand().schedule();
                        }));
    }

    public static void configureTestingControls(int port, Drivetrain drivetrain, Elevator elevator, Arm arm,
            Intake intake,
            Climber climber, LEDs leds) {

        CommandXboxController controller = new CommandXboxController(port);
        controller.b().toggleOnTrue(leds.requestStateCommand(LEDState.DEMO_RED));
        controller.y().toggleOnTrue(leds.requestStateCommand(LEDState.DEMO_GOLD));
    }
}
