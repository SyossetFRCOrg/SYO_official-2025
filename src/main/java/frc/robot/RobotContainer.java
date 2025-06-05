package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.camera2Name;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ControllerRumbleCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ReefAlignController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.Superstructure.CurrentSuperState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIOTalonFX;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Vision vision;
  private final Drive drive;
  private final Elevator elevator;

  private final Intake intake;
  private final Wrist wrist;
  private final Superstructure superstructure;

  private final Climber climber;

  // Controller
  private final XboxController controller = new XboxController(0);
  private final XboxController buttonboard = new XboxController(1);

  private final UsbCamera climbCam;

  //   private final AutoSelector autoSelector = new AutoSelector("Auto");

  //   private ReefAlignController autoAlignController;
  private ReefAlignController reefAlignController;

  //   // Dashboard inputs
  //   private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {

    drive =
        new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));

    elevator = new Elevator(new ElevatorIOTalonFX());
    wrist = new Wrist(new WristIOTalonFX());
    intake = new Intake(new IntakeIOTalonFX());

    vision =
        new Vision(
            drive::addVisionMeasurement,
            drive,
            new VisionIOLimelight(camera0Name, drive::getRotation),
            new VisionIOLimelight(camera1Name, drive::getRotation),
            new VisionIOLimelight(camera2Name, drive::getRotation));

    reefAlignController = new ReefAlignController(drive, () -> false, () -> false);

    climber = new Climber(new ClimberIOSparkMax());

    superstructure = new Superstructure(drive, elevator, wrist, this, intake);

    // configureAutos();

    // Configure the button bindings
    configureButtonBindings();

    climbCam = CameraServer.startAutomaticCapture();
    climbCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    climbCam.setResolution(80, 60);
  }

  //   private void configureAutos() {
  //     AutoFactory autoBuilder = new AutoFactory(drive, superstructure,
  // autoSelector::getResponses);

  //     // Add autos
  //     autoSelector.addRoutine(
  //         "4 Coral flexible",
  //         List.of(
  //             new AutoQuestion(
  //                 "Starting location?",
  //                 List.of(
  //                     AutoQuestionResponse.FAR_LEFT,
  //                     AutoQuestionResponse.MID_LEFT,
  //                     AutoQuestionResponse.MID_RIGHT,
  //                     AutoQuestionResponse.FAR_RIGHT)),
  //             new AutoQuestion(
  //                 "First Scoring Location?",
  //                 List.of(
  //                     AutoQuestionResponse.A,
  //                     AutoQuestionResponse.B,
  //                     AutoQuestionResponse.C,
  //                     AutoQuestionResponse.D,
  //                     AutoQuestionResponse.E,
  //                     AutoQuestionResponse.F,
  //                     AutoQuestionResponse.G,
  //                     AutoQuestionResponse.H,
  //                     AutoQuestionResponse.I,
  //                     AutoQuestionResponse.J,
  //                     AutoQuestionResponse.K,
  //                     AutoQuestionResponse.L)),
  //             new AutoQuestion(
  //                 "First Scoring Height?",
  //                 List.of(AutoQuestionResponse.L2, AutoQuestionResponse.L3,
  // AutoQuestionResponse.L4)),
  //             new AutoQuestion(
  //                 "First Intaking Position?",
  //                 List.of(
  //                     AutoQuestionResponse.RIGHT_FORWARD_CORALSTATION,
  //                     AutoQuestionResponse.RIGHT_BACK_CORALSTATION,
  //                     AutoQuestionResponse.LEFT_FORWARD_CORALSTATION,
  //                     AutoQuestionResponse.LEFT_BACK_CORALSTATION)),
  //             new AutoQuestion(
  //                 "Second Scoring Location?",
  //                 List.of(
  //                     AutoQuestionResponse.A,
  //                     AutoQuestiosponse.E,
  //                     AutoQuestionResponse.F,
  //                     AutoQuestionResponse.G,
  //                     AutoQuestionResponse.H,
  //                     AutoQuestionResponse.I,
  //                     AutoQuestionResponse.J,
  //                     AutoQuestionResponse.K,
  //                     AutoQuestionResponse.B,
  //                     AutoQuestionResponse.C,
  //                     AutoQuestionResponse.D,
  //                     AutoQuestionRense.L)),
  //             new AutoQuestion(
  //                 "Second Scoring Height?",
  //                 List.of(AutoQuestionResponse.L2, AutoQuestionResponse.L3,
  // AutoQuestionResponse.L4)),
  //             new AutoQuestion(
  //                 "Second Intaking Position?",
  //                 List.of(
  //                     AutoQuestionResponse.RIGHT_FORWARD_CORALSTATION,
  //                     AutoQuestionResponse.RIGHT_BACK_CORALSTATION,
  //                     AutoQuestionResponse.LEFT_FORWARD_CORALSTATION,
  //                     AutoQuestionResponse.LEFT_BACK_CORALSTATION)),
  //             new AutoQuestion(
  //                 "Third Scoring Location?",
  //                 List.of(
  //                     AutoQuestionResponse.A,
  //                     AutoQuestionResponse.B,
  //                     AutoQuestionResponse.C,
  //                     AutoQuestionResponse.D,
  //                     AutoQuestionResponse.E,
  //                     AutoQuestionResponse.F,
  //                     AutoQuestionResponse.G,
  //                     AutoQuestionResponse.H,
  //                     AutoQuestionResponse.I,
  //                     AutoQuestionResponse.J,
  //                     AutoQuestionResponse.K,
  //                     AutoQuestionResponse.L)),
  //             new AutoQuestion(
  //                 "Third Scoring Height?",
  //                 List.of(AutoQuestionResponse.L2, AutoQuestionResponse.L3,
  // AutoQuestionResponse.L4)),
  //             new AutoQuestion(
  //                 "Third Intaking Position?",
  //                 List.of(
  //                     AutoQuestionResponse.RIGHT_FORWARD_CORALSTATION,
  //                     AutoQuestionResponse.RIGHT_BACK_CORALSTATION,
  //                     AutoQuestionResponse.LEFT_FORWARD_CORALSTATION,
  //                     AutoQuestionResponse.LEFT_BACK_CORALSTATION)),
  //             new AutoQuestion(
  //                 "Fourth Scoring Location?",
  //                 List.of(
  //                     AutoQuestionResponse.A,
  //                     AutoQuestionResponse.B,
  //                     AutoQuestionResponse.C,
  //                     AutoQuestionResponse.D,
  //                     AutoQuestionResponse.E,
  //                     AutoQuestionResponse.F,
  //                     AutoQuestionResponse.G,
  //                     AutoQuestionResponse.H,
  //                     AutoQuestionResponse.I,
  //                     AutoQuestionResponse.J,
  //                     AutoQuestionResponse.K,
  //                     AutoQuestionResponse.L)),
  //             new AutoQuestion(
  //                 "Fourth Scoring Height?",
  //                 List.of(AutoQuestionResponse.L2, AutoQuestionResponse.L3,
  // AutoQuestionResponse.L4)),
  //             new AutoQuestion(
  //                 "Fourth Intaking Position?",
  //                 List.of(
  //                     AutoQuestionResponse.RIGHT_FORWARD_CORALSTATION,
  //                     AutoQuestionResponse.RIGHT_BACK_CORALSTATION,
  //                     AutoQuestionResponse.LEFT_FORWARD_CORALSTATION,
  //                     AutoQuestionResponse.LEFT_BACK_CORALSTATION))),
  //         autoBuilder.FourAuto()
  //         // autoBuilder.createIdleCommand()
  //         );

  //     autoSelector.addRoutine(
  //         "3 Coral flexible",
  //         List.of(
  //             new AutoQuestion(
  //                 "Starting location?",
  //                 List.of(
  //                     AutoQuestionResponse.FAR_LEFT,
  //                     AutoQuestionResponse.MID_LEFT,
  //                     AutoQuestionResponse.MID_RIGHT,
  //                     AutoQuestionResponse.FAR_RIGHT)),
  //             new AutoQuestion(
  //                 "First Scoring Location?",
  //                 List.of(
  //                     AutoQuestionResponse.A,
  //                     AutoQuestionResponse.B,
  //                     AutoQuestionResponse.C,
  //                     AutoQuestionResponse.D,
  //                     AutoQuestionResponse.E,
  //                     AutoQuestionResponse.F,
  //                     AutoQuestionResponse.G,
  //                     AutoQuestionResponse.H,
  //                     AutoQuestionResponse.I,
  //                     AutoQuestionResponse.J,
  //                     AutoQuestionResponse.K,
  //                     AutoQuestionResponse.L)),
  //             new AutoQuestion(
  //                 "First Scoring Height?",
  //                 List.of(AutoQuestionResponse.L2, AutoQuestionResponse.L3,
  // AutoQuestionResponse.L4)),
  //             new AutoQuestion(
  //                 "First Intaking Position?",
  //                 List.of(
  //                     AutoQuestionResponse.RIGHT_FORWARD_CORALSTATION,
  //                     AutoQuestionResponse.RIGHT_BACK_CORALSTATION,
  //                     AutoQuestionResponse.LEFT_FORWARD_CORALSTATION,
  //                     AutoQuestionResponse.LEFT_BACK_CORALSTATION)),
  //             new AutoQuestion(
  //                 "Second Scoring Location?",
  //                 List.of(
  //                     AutoQuestionResponse.A,
  //                     AutoQuestionResponse.B,
  //                     AutoQuestionResponse.C,
  //                     AutoQuestionResponse.D,
  //                     AutoQuestionResponse.E,
  //                     AutoQuestionResponse.F,
  //                     AutoQuestionResponse.G,
  //                     AutoQuestionResponse.H,
  //                     AutoQuestionResponse.I,
  //                     AutoQuestionResponse.J,
  //                     AutoQuestionResponse.K,
  //                     AutoQuestionResponse.L)),
  //             new AutoQuestion(
  //                 "Second Scoring Height?",
  //                 List.of(AutoQuestionResponse.L2, AutoQuestionResponse.L3,
  // AutoQuestionResponse.L4)),
  //             new AutoQuestion(
  //                 "Second Intaking Position?",
  //                 List.of(
  //                     AutoQuestionResponse.RIGHT_FORWARD_CORALSTATION,
  //                     AutoQuestionResponse.RIGHT_BACK_CORALSTATION,
  //                     AutoQuestionResponse.LEFT_FORWARD_CORALSTATION,
  //                     AutoQuestionResponse.LEFT_BACK_CORALSTATION)),
  //             new AutoQuestion(
  //                 "Third Scoring Location?",
  //                 List.of(
  //                     AutoQuestionResponse.A,
  //                     AutoQuestionResponse.B,
  //                     AutoQuestionResponse.C,
  //                     AutoQuestionResponse.D,
  //                     AutoQuestionResponse.E,
  //                     AutoQuestionResponse.F,
  //                     AutoQuestionResponse.G,
  //                     AutoQuestionResponse.H,
  //                     AutoQuestionResponse.I,
  //                     AutoQuestionResponse.J,
  //                     AutoQuestionResponse.K,
  //                     AutoQuestionResponse.L)),
  //             new AutoQuestion(
  //                 "Third Scoring Height?",
  //                 List.of(AutoQuestionResponse.L2, AutoQuestionResponse.L3,
  // AutoQuestionResponse.L4)),
  //             new AutoQuestion(
  //                 "Third Intaking Position?",
  //                 List.of(
  //                     AutoQuestionResponse.RIGHT_FORWARD_CORALSTATION,
  //                     AutoQuestionResponse.RIGHT_BACK_CORALSTATION,
  //                     AutoQuestionResponse.LEFT_FORWARD_CORALSTATION,
  //                     AutoQuestionResponse.LEFT_BACK_CORALSTATION))),
  //         autoBuilder.ThreeAuto());

  //     autoSelector.addRoutine(
  //         "2 Coral flexible",
  //         List.of(
  //             new AutoQuestion(
  //                 "Starting location?",
  //                 List.of(
  //                     AutoQuestionResponse.FAR_LEFT,
  //                     AutoQuestionResponse.MID_LEFT,
  //                     AutoQuestionResponse.MID_RIGHT,
  //                     AutoQuestionResponse.FAR_RIGHT)),
  //             new AutoQuestion(
  //                 "First Scoring Location?",
  //                 List.of(
  //                     AutoQuestionResponse.A,
  //                     AutoQuestionResponse.B,
  //                     AutoQuestionResponse.C,
  //                     AutoQuestionResponse.D,
  //                     AutoQuestionResponse.E,
  //                     AutoQuestionResponse.F,
  //                     AutoQuestionResponse.G,
  //                     AutoQuestionResponse.H,
  //                     AutoQuestionResponse.I,
  //                     AutoQuestionResponse.J,
  //                     AutoQuestionResponse.K,
  //                     AutoQuestionResponse.L)),
  //             new AutoQuestion(
  //                 "First Scoring Height?",
  //                 List.of(AutoQuestionResponse.L2, AutoQuestionResponse.L3,
  // AutoQuestionResponse.L4)),
  //             new AutoQuestion(
  //                 "First Intaking Position?",
  //                 List.of(
  //                     AutoQuestionResponse.RIGHT_FORWARD_CORALSTATION,
  //                     AutoQuestionResponse.RIGHT_BACK_CORALSTATION,
  //                     AutoQuestionResponse.LEFT_FORWARD_CORALSTATION,
  //                     AutoQuestionResponse.LEFT_BACK_CORALSTATION)),
  //             new AutoQuestion(
  //                 "Second Scoring Location?",
  //                 List.of(
  //                     AutoQuestionResponse.A,
  //                     AutoQuestionResponse.B,
  //                     AutoQuestionResponse.C,
  //                     AutoQuestionResponse.D,
  //                     AutoQuestionResponse.E,
  //                     AutoQuestionResponse.F,
  //                     AutoQuestionResponse.G,
  //                     AutoQuestionResponse.H,
  //                     AutoQuestionResponse.I,
  //                     AutoQuestionResponse.J,
  //                     AutoQuestionResponse.K,
  //                     AutoQuestionResponse.L)),
  //             new AutoQuestion(
  //                 "Second Scoring Height?",
  //                 List.of(AutoQuestionResponse.L2, AutoQuestionResponse.L3,
  // AutoQuestionResponse.L4)),
  //             new AutoQuestion(
  //                 "Second Intaking Position?",
  //                 List.of(
  //                     AutoQuestionResponse.RIGHT_FORWARD_CORALSTATION,
  //                     AutoQuestionResponse.RIGHT_BACK_CORALSTATION,
  //                     AutoQuestionResponse.LEFT_FORWARD_CORALSTATION,
  //                     AutoQuestionResponse.LEFT_BACK_CORALSTATION))),
  //         autoBuilder.TwoAuto());
  //   }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Default command, normal field-relative drive
    Trigger stow = new Trigger(() -> superstructure.getCurrentSuperState() == CurrentSuperState.STOWED);

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to nearest coral station's angle when A button is held
    // also go towards it, still allowing for driver translation

    Trigger AArightBumper =
        new Trigger(
            () ->
                controller.getRawButton(6)
                    && RobotState.getInstance().isIntakeAutoAiming()
                    && !controller.getLeftBumperButton());
    AArightBumper.onTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.INTAKE));

    AArightBumper.onFalse(superstructure.setWantedSuperStateCommand(WantedSuperState.STOW));

    AArightBumper.whileTrue(
        DriveCommands.joystickDriveCoralStation(
            drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    Trigger noAArightBumper =
        new Trigger(
            () ->
                controller.getRawButton(6)
                    && !RobotState.getInstance().isIntakeAutoAiming()
                    && !controller.getLeftBumperButton());
    noAArightBumper.onTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.INTAKE));
    noAArightBumper.onFalse(superstructure.setWantedSuperStateCommand(WantedSuperState.STOW));

    Trigger AArightBumperLowIntake =
        new Trigger(
            () ->
                controller.getRawButton(6)
                    && RobotState.getInstance().isIntakeAutoAiming()
                    && controller.getLeftBumperButton());
    AArightBumperLowIntake.onTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.INTAKELOW));

    AArightBumperLowIntake.onFalse(superstructure.setWantedSuperStateCommand(WantedSuperState.STOW));

    AArightBumperLowIntake.whileTrue(
        DriveCommands.joystickDriveCoralStation(
            drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    Trigger noAArightBumperLowIntake =
        new Trigger(
            () ->
                controller.getRawButton(6)
                    && !RobotState.getInstance().isIntakeAutoAiming()
                    && controller.getLeftBumperButton());
    noAArightBumperLowIntake.onTrue(
        superstructure.setWantedSuperStateCommand(WantedSuperState.INTAKELOW));
    noAArightBumperLowIntake.onFalse(superstructure.setWantedSuperStateCommand(WantedSuperState.STOW));

    Trigger noAAL1 = new Trigger(() -> controller.getBButton());

    noAAL1
        .onTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.L1PREPARE))
        .onFalse(
            new WaitCommand(1)
                .deadlineFor(superstructure.setWantedSuperStateCommand(WantedSuperState.L1))
                .andThen(superstructure.setWantedSuperStateCommand(WantedSuperState.STOW)));

    Trigger AAL2 =
        new Trigger(() -> controller.getAButton() && RobotState.getInstance().isReefAutoAligning());

    AAL2.onTrue(
            superstructure
                .setWantedSuperStateCommand(WantedSuperState.L2)
                .andThen(
                    new InstantCommand(
                        () -> {
                          reefAlignController =
                              new ReefAlignController(
                                  drive,
                                  () ->
                                      RobotState.getInstance()
                                              .getDistanceToNearestReef(drive.getPose())
                                          < 1.0,
                                  () -> false);
                        })))
        .onFalse(superstructure.setWantedSuperStateCommand(WantedSuperState.STOW))
        .whileTrue(
            new InstantCommand(
                    () -> {
                      drive.runVelocity(reefAlignController.update().get());
                    },
                    drive)
                .repeatedly()
                .until(() -> reefAlignController.atGoal())
                .andThen(
                    new ParallelRaceGroup(
                        new ControllerRumbleCommand(controller, () -> true), new WaitCommand(.4))));

    Trigger noAAL2 =
        new Trigger(
            () -> controller.getAButton() && !RobotState.getInstance().isReefAutoAligning());

    noAAL2
        .onTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.L2PREPARE))
        .onFalse(
            new WaitCommand(1)
                .deadlineFor(superstructure.setWantedSuperStateCommand(WantedSuperState.L2))
                .andThen(superstructure.setWantedSuperStateCommand(WantedSuperState.STOW)));

    Trigger AAimL2 =
        new Trigger(
            () ->
                (controller.getAButton()
                    && !RobotState.getInstance().isReefAutoAligning()
                    && RobotState.getInstance().isReefAutoAiming()));

    AAimL2.onTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.L2PREPARE))
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> RobotState.getInstance().getNearestReefPose(drive.getPose()).getRotation()))
        .onFalse(
            new WaitCommand(1)
                .deadlineFor(superstructure.setWantedSuperStateCommand(WantedSuperState.L2))
                .andThen(superstructure.setWantedSuperStateCommand(WantedSuperState.STOW)));

    Trigger AAL3 =
        new Trigger(
            () -> (controller.getXButton() && RobotState.getInstance().isReefAutoAligning()));

    AAL3.onTrue(
            superstructure
                .setWantedSuperStateCommand(WantedSuperState.L3)
                .andThen(
                    new InstantCommand(
                        () -> {
                          reefAlignController =
                              new ReefAlignController(
                                  drive,
                                  () ->
                                      RobotState.getInstance()
                                              .getDistanceToNearestReef(drive.getPose())
                                          < 1.0,
                                  () -> false);
                        })))
        .onFalse(superstructure.setWantedSuperStateCommand(WantedSuperState.STOW))
        .onFalse(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()))
        .whileTrue(
            new InstantCommand(
                    () -> {
                      drive.runVelocity(reefAlignController.update().get());
                    },
                    drive)
                .repeatedly()
                .until(() -> reefAlignController.atGoal())
                .andThen(
                    new ParallelRaceGroup(
                        new ControllerRumbleCommand(controller, () -> true), new WaitCommand(.4))));

    Trigger noAAL3 =
        new Trigger(
            () -> controller.getXButton() && !RobotState.getInstance().isReefAutoAligning());

    noAAL3
        .onTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.L3PREPARE))
        .onFalse(
            new WaitCommand(1)
                .deadlineFor(superstructure.setWantedSuperStateCommand(WantedSuperState.L3))
                .andThen(superstructure.setWantedSuperStateCommand(WantedSuperState.STOW)));

    Trigger AAimL3 =
        new Trigger(
            () ->
                (controller.getYButton()
                    && !RobotState.getInstance().isReefAutoAligning()
                    && RobotState.getInstance().isReefAutoAiming()));

    AAimL3.onTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.L3PREPARE))
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> RobotState.getInstance().getNearestReefPose(drive.getPose()).getRotation()))
        .onFalse(
            new WaitCommand(1)
                .deadlineFor(superstructure.setWantedSuperStateCommand(WantedSuperState.L3))
                .andThen(superstructure.setWantedSuperStateCommand(WantedSuperState.STOW)));

    Trigger AAL4 =
        new Trigger(
            () -> (controller.getYButton() && RobotState.getInstance().isReefAutoAligning()));

    AAL4.onTrue(
            superstructure
                .setWantedSuperStateCommand(WantedSuperState.L4)
                .andThen(
                    new InstantCommand(
                        () -> {
                          reefAlignController =
                              new ReefAlignController(
                                  drive,
                                  () ->
                                      RobotState.getInstance()
                                              .getDistanceToNearestReef(drive.getPose())
                                          < 1.0,
                                  () -> false);
                        })))
        .onFalse(superstructure.setWantedSuperStateCommand(WantedSuperState.STOW))
        .onFalse(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()))
        .whileTrue(
            new InstantCommand(
                    () -> {
                      drive.runVelocity(reefAlignController.update().get());
                    },
                    drive)
                .repeatedly()
                .until(() -> reefAlignController.atGoal())
                .andThen(
                    new ParallelRaceGroup(
                        new ControllerRumbleCommand(controller, () -> true), new WaitCommand(.4))));

    Trigger noAAL4 =
        new Trigger(
            () ->
                controller.getYButton()
                    && !RobotState.getInstance().isReefAutoAligning()
                    && !RobotState.getInstance().isReefAutoAiming());

    noAAL4
        .onTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.L4PREPARE))
        .onFalse(
            new WaitCommand(1)
                .deadlineFor(superstructure.setWantedSuperStateCommand(WantedSuperState.L4))
                .andThen(superstructure.setWantedSuperStateCommand(WantedSuperState.STOW)));

    Trigger AAimL4 =
        new Trigger(
            () ->
                (controller.getYButton()
                    && !RobotState.getInstance().isReefAutoAligning()
                    && RobotState.getInstance().isReefAutoAiming()));

    AAimL4.onTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.L4PREPARE))
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> RobotState.getInstance().getNearestReefPose(drive.getPose()).getRotation()))
        .onFalse(
            new WaitCommand(1)
                .deadlineFor(superstructure.setWantedSuperStateCommand(WantedSuperState.L4))
                .andThen(superstructure.setWantedSuperStateCommand(WantedSuperState.STOW)));

    Trigger WheelCharacterize = new Trigger(() -> buttonboard.getRawAxis(2) > .5);
    WheelCharacterize.onTrue(
        waitSeconds(90).deadlineFor(DriveCommands.wheelRadiusCharacterization(drive)));

    Trigger LLToggle = new Trigger(() -> buttonboard.getRawButton(1));

    LLToggle.onTrue(
        new InstantCommand(
            () ->
                RobotState.getInstance()
                    .setAddingVision(!RobotState.getInstance().isAddingVision())));

    Trigger reefAAToggle = new Trigger(() -> buttonboard.getRawButton(3));

    reefAAToggle.onTrue(
        new InstantCommand(
            () ->
                RobotState.getInstance()
                    .setReefAutoAligning(!RobotState.getInstance().isReefAutoAligning())));

    Trigger tuningPoseToggle = new Trigger(() -> buttonboard.getRawButton(2));

    tuningPoseToggle.onTrue(
        new InstantCommand(
            () -> {

              // if it is null, make it our current pose. If it is not null, make it null.
              if (RobotState.getInstance().getTuningTempPose() == null) {
                RobotState.getInstance().setTuningTempPose(drive.getPose());
              } else if (RobotState.getInstance().getTuningTempPose() != null) {
                RobotState.getInstance().setTuningTempPose(null);
              }
            }));

    Trigger intakeAutoAimToggle = new Trigger(() -> buttonboard.getRawButton(4));
    intakeAutoAimToggle.onTrue(
        new InstantCommand(
            () ->
                RobotState.getInstance()
                    .setIntakeAutoAiming(!RobotState.getInstance().isIntakeAutoAiming())));

    Trigger reefAutoAimToggle = new Trigger(() -> buttonboard.getRawButton(5));
    reefAutoAimToggle.onTrue(
        new InstantCommand(
            () ->
                RobotState.getInstance()
                    .setReefAutoAiming(!RobotState.getInstance().isReefAutoAiming())));

    Trigger elevatorUpManual = new Trigger(() -> buttonboard.getRawButton(6));
    elevatorUpManual.whileTrue(
        new InstantCommand(
                () -> {
                  elevator.setHeight(elevator.getHeight() - .05);
                })
            .repeatedly()
            .ignoringDisable(true));

    Trigger elevatorDownManual = new Trigger(() -> buttonboard.getRawAxis(3) > .5);
    elevatorDownManual.whileTrue(
        new InstantCommand(
                () -> {
                  elevator.setHeight(elevator.getHeight() + .05);
                })
            .repeatedly()
            .ignoringDisable(true));

    Trigger wristUpManual = new Trigger(() -> controller.getPOV() == 90);
    // reseting wrist and elevator encoder to their "zero" positions
    // must physically properly be "zeroed" for this to have desired effect
    wristUpManual.whileTrue(
        new InstantCommand(
                () -> {
                  wrist.resetPosition(wrist.getPosition() + .01);
                })
            .repeatedly()
            .ignoringDisable(true));

    Trigger wristDownManual = new Trigger(() -> controller.getPOV() == 270);
    // reseting wrist and elevator encoder to their "zero" positions
    // must physically properly be "zeroed" for this to have desired effect
    wristDownManual.whileTrue(
        new InstantCommand(
                () -> {
                  wrist.resetPosition(wrist.getPosition() - .01);
                })
            .repeatedly()
            .ignoringDisable(true));

    // switch the stick of the reef (on the same face) that is being aligned to.
    // for an easy toggle that can be done while aligning (not letting go of alignment button)
    // repeatedly, safely.
    Trigger toggleReefSide =
        new Trigger(() -> controller.getLeftBumperButton() && !controller.getRightBumperButton());
    toggleReefSide.onTrue(
        new InstantCommand(
            () ->
                reefAlignController =
                    new ReefAlignController(
                        drive,
                        () ->
                            RobotState.getInstance().getDistanceToNearestReef(drive.getPose())
                                < 1.0,
                        () -> true)));

    // Reset gyro to 0° when the menu looking button is pressed
    Trigger resetPoseTrigger = new Trigger(() -> controller.getRawButton(8));
    resetPoseTrigger.onTrue(
        Commands.runOnce(
                () ->
                    drive.setPose(
                        new Pose2d(
                            drive.getPose().getX(),
                            drive.getPose().getY(),
                            DriverStation.getAlliance().get() == Alliance.Blue
                                ? Rotation2d.fromRadians(0)
                                : Rotation2d.fromDegrees(180))),
                drive)
            .ignoringDisable(true));

    Trigger rightTrigger = new Trigger(() -> controller.getRightTriggerAxis() > .5);

    rightTrigger
        .onTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.L3L4ALGAE))
        .onFalse(superstructure.setWantedSuperStateCommand(WantedSuperState.STOW));

    Trigger leftTrigger = new Trigger(() -> controller.getLeftTriggerAxis() > .5);

    leftTrigger
        .onTrue(superstructure.setWantedSuperStateCommand(WantedSuperState.L2L3ALGAE))
        .onFalse(superstructure.setWantedSuperStateCommand(WantedSuperState.STOW));

    Trigger climberUpTrigger = new Trigger(() -> controller.getPOV() == 0);
    climberUpTrigger
        .onTrue(
            // moving elevator up during climb, we want the robot to lean backwards
            // so the chain doesn't touch the elevator (hopefully)
            superstructure
                .setWantedSuperStateCommand(WantedSuperState.L1PREPARE)
                .alongWith(climber.setMotorVoltage(-12)))
        .onFalse(climber.setMotorVoltage(0));

    Trigger climberDownTrigger = new Trigger(() -> controller.getPOV() == 180);
    climberDownTrigger
        .onTrue(
            superstructure
                .setWantedSuperStateCommand(WantedSuperState.L1PREPARE)
                .alongWith(climber.setMotorVoltage(12)))
        .onFalse(climber.setMotorVoltage(0));
  }

  //   /**
  //    * Use this to pass the autonomous command to the main {@link Robot} class.
  //    *
  //    * @return the command to run in autonomous
  //    */
  //   public Command getAutonomousCommand() {
  //     return autoChooser.get();
  //   }

  public ReefAlignController getReefAlignController() {
    return reefAlignController;
  }

  public Drive getDrive() {
    return drive;
  }

  //   /**
  //    * Use this to pass the autonomous command to the main {@link Robot} class.
  //    *
  //    * @return the command to run in autonomous
  //    */
  //   public Command getAutonomousCommand() {
  //     return autoSelector.getCommand();
  //   }
  public Superstructure getSuperstructure() {
    return superstructure;
  }
}
