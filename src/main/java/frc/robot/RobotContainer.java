package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.AutoSelector.AutoQuestion;
import frc.robot.AutoSelector.AutoQuestionResponse;
import frc.robot.autos.AutoFactory;
import frc.robot.commands.ControllerRumbleCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ReefAlignController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIOSparkMax;
import frc.robot.util.Container;

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

  // Controller
  private final XboxController controller = new XboxController(0);

  private final AutoSelector autoSelector = new AutoSelector("Auto");

  //   private ReefAlignController autoAlignController;
  private ReefAlignController reefAlignController;

  //   // Dashboard inputs
  //   private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    drive =
        new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));

    elevator = new Elevator(new ElevatorIOSparkMax());
    wrist = new Wrist(new WristIOSparkMax());
    intake = new Intake(new IntakeIOTalonFX());

    vision =
        new Vision(
            drive::addVisionMeasurement,
            drive,
            new VisionIOLimelight(camera0Name, drive::getRotation),
            new VisionIOLimelight(camera1Name, drive::getRotation));

    reefAlignController = new ReefAlignController(drive, () -> false, () -> false);

    superstructure = new Superstructure(drive, elevator, wrist, this);

    configureAutos();

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureAutos() {
    AutoFactory autoBuilder = new AutoFactory(drive, superstructure, autoSelector::getResponses);

    // Add autos
    autoSelector.addRoutine(
        "4 Coral flexible",
        List.of(
            new AutoQuestion(
                "Starting location?",
                List.of(
                    AutoQuestionResponse.FAR_LEFT,
                    AutoQuestionResponse.MID_LEFT,
                    AutoQuestionResponse.MID_RIGHT,
                    AutoQuestionResponse.FAR_RIGHT)),
            new AutoQuestion(
                "First Scoring Location?",
                List.of(AutoQuestionResponse.A, AutoQuestionResponse.B, AutoQuestionResponse.C, AutoQuestionResponse.D, AutoQuestionResponse.E, AutoQuestionResponse.F, AutoQuestionResponse.G, AutoQuestionResponse.H, AutoQuestionResponse.I, AutoQuestionResponse.J, AutoQuestionResponse.K, AutoQuestionResponse.L)),
            new AutoQuestion(
                "First Scoring Height?", List.of(AutoQuestionResponse.L2, AutoQuestionResponse.L3, AutoQuestionResponse.L4)),
            new AutoQuestion(
                "First Intaking Position?", List.of(AutoQuestionResponse.RIGHT_FORWARD_CORALSTATION, AutoQuestionResponse.RIGHT_BACK_CORALSTATION, AutoQuestionResponse.LEFT_FORWARD_CORALSTATION, AutoQuestionResponse.LEFT_BACK_CORALSTATION)),
            new AutoQuestion(
                "Second Scoring Location?",
                List.of(AutoQuestionResponse.A, AutoQuestionResponse.B, AutoQuestionResponse.C, AutoQuestionResponse.D, AutoQuestionResponse.E, AutoQuestionResponse.F, AutoQuestionResponse.G, AutoQuestionResponse.H, AutoQuestionResponse.I, AutoQuestionResponse.J, AutoQuestionResponse.K, AutoQuestionResponse.L)),
            new AutoQuestion(
                "Second Scoring Height?", List.of(AutoQuestionResponse.L2, AutoQuestionResponse.L3, AutoQuestionResponse.L4)),
            new AutoQuestion(
                "Second Intaking Position?", List.of(AutoQuestionResponse.RIGHT_FORWARD_CORALSTATION, AutoQuestionResponse.RIGHT_BACK_CORALSTATION, AutoQuestionResponse.LEFT_FORWARD_CORALSTATION, AutoQuestionResponse.LEFT_BACK_CORALSTATION)),
            new AutoQuestion(
                "Third Scoring Location?",
                List.of(AutoQuestionResponse.A, AutoQuestionResponse.B, AutoQuestionResponse.C, AutoQuestionResponse.D, AutoQuestionResponse.E, AutoQuestionResponse.F, AutoQuestionResponse.G, AutoQuestionResponse.H, AutoQuestionResponse.I, AutoQuestionResponse.J, AutoQuestionResponse.K, AutoQuestionResponse.L)),
            new AutoQuestion(
                "Third Scoring Height?", List.of(AutoQuestionResponse.L2, AutoQuestionResponse.L3, AutoQuestionResponse.L4)),
            new AutoQuestion(
                "Third Intaking Position?", List.of(AutoQuestionResponse.RIGHT_FORWARD_CORALSTATION, AutoQuestionResponse.RIGHT_BACK_CORALSTATION, AutoQuestionResponse.LEFT_FORWARD_CORALSTATION, AutoQuestionResponse.LEFT_BACK_CORALSTATION)),
            new AutoQuestion(
                "Fourth Scoring Location?",
                List.of(AutoQuestionResponse.A, AutoQuestionResponse.B, AutoQuestionResponse.C, AutoQuestionResponse.D, AutoQuestionResponse.E, AutoQuestionResponse.F, AutoQuestionResponse.G, AutoQuestionResponse.H, AutoQuestionResponse.I, AutoQuestionResponse.J, AutoQuestionResponse.K, AutoQuestionResponse.L)),
            new AutoQuestion(
                "Fourth Scoring Height?", List.of(AutoQuestionResponse.L2, AutoQuestionResponse.L3, AutoQuestionResponse.L4)),
            new AutoQuestion(
                "Fourth Intaking Position?", List.of(AutoQuestionResponse.RIGHT_FORWARD_CORALSTATION, AutoQuestionResponse.RIGHT_BACK_CORALSTATION, AutoQuestionResponse.LEFT_FORWARD_CORALSTATION, AutoQuestionResponse.LEFT_BACK_CORALSTATION))),
                        
        autoBuilder.FourAuto());

        autoSelector.addRoutine(
            "4 Coral flexible",
            List.of(
                new AutoQuestion(
                    "Starting location?",
                    List.of(
                        AutoQuestionResponse.FAR_LEFT,
                        AutoQuestionResponse.MID_LEFT,
                        AutoQuestionResponse.MID_RIGHT,
                        AutoQuestionResponse.FAR_RIGHT)),
                new AutoQuestion(
                    "First Scoring Location?",
                    List.of(AutoQuestionResponse.A, AutoQuestionResponse.B, AutoQuestionResponse.C, AutoQuestionResponse.D, AutoQuestionResponse.E, AutoQuestionResponse.F, AutoQuestionResponse.G, AutoQuestionResponse.H, AutoQuestionResponse.I, AutoQuestionResponse.J, AutoQuestionResponse.K, AutoQuestionResponse.L)),
                new AutoQuestion(
                    "First Scoring Height?", List.of(AutoQuestionResponse.L2, AutoQuestionResponse.L3, AutoQuestionResponse.L4)),
                new AutoQuestion(
                    "First Intaking Position?", List.of(AutoQuestionResponse.RIGHT_FORWARD_CORALSTATION, AutoQuestionResponse.RIGHT_BACK_CORALSTATION, AutoQuestionResponse.LEFT_FORWARD_CORALSTATION, AutoQuestionResponse.LEFT_BACK_CORALSTATION)),
                new AutoQuestion(
                    "Second Scoring Location?",
                    List.of(AutoQuestionResponse.A, AutoQuestionResponse.B, AutoQuestionResponse.C, AutoQuestionResponse.D, AutoQuestionResponse.E, AutoQuestionResponse.F, AutoQuestionResponse.G, AutoQuestionResponse.H, AutoQuestionResponse.I, AutoQuestionResponse.J, AutoQuestionResponse.K, AutoQuestionResponse.L)),
                new AutoQuestion(
                    "Second Scoring Height?", List.of(AutoQuestionResponse.L2, AutoQuestionResponse.L3, AutoQuestionResponse.L4)),
                new AutoQuestion(
                    "Second Intaking Position?", List.of(AutoQuestionResponse.RIGHT_FORWARD_CORALSTATION, AutoQuestionResponse.RIGHT_BACK_CORALSTATION, AutoQuestionResponse.LEFT_FORWARD_CORALSTATION, AutoQuestionResponse.LEFT_BACK_CORALSTATION)),
                new AutoQuestion(
                    "Third Scoring Location?",
                    List.of(AutoQuestionResponse.A, AutoQuestionResponse.B, AutoQuestionResponse.C, AutoQuestionResponse.D, AutoQuestionResponse.E, AutoQuestionResponse.F, AutoQuestionResponse.G, AutoQuestionResponse.H, AutoQuestionResponse.I, AutoQuestionResponse.J, AutoQuestionResponse.K, AutoQuestionResponse.L)),
                new AutoQuestion(
                    "Third Scoring Height?", List.of(AutoQuestionResponse.L2, AutoQuestionResponse.L3, AutoQuestionResponse.L4)),
                new AutoQuestion(
                    "Third Intaking Position?", List.of(AutoQuestionResponse.RIGHT_FORWARD_CORALSTATION, AutoQuestionResponse.RIGHT_BACK_CORALSTATION, AutoQuestionResponse.LEFT_FORWARD_CORALSTATION, AutoQuestionResponse.LEFT_BACK_CORALSTATION))
            ),        
            autoBuilder.ThreeAuto());

            autoSelector.addRoutine(
            "4 Coral flexible",
            List.of(
                new AutoQuestion(
                    "Starting location?",
                    List.of(
                        AutoQuestionResponse.FAR_LEFT,
                        AutoQuestionResponse.MID_LEFT,
                        AutoQuestionResponse.MID_RIGHT,
                        AutoQuestionResponse.FAR_RIGHT)),
                new AutoQuestion(
                    "First Scoring Location?",
                    List.of(AutoQuestionResponse.A, AutoQuestionResponse.B, AutoQuestionResponse.C, AutoQuestionResponse.D, AutoQuestionResponse.E, AutoQuestionResponse.F, AutoQuestionResponse.G, AutoQuestionResponse.H, AutoQuestionResponse.I, AutoQuestionResponse.J, AutoQuestionResponse.K, AutoQuestionResponse.L)),
                new AutoQuestion(
                    "First Scoring Height?", List.of(AutoQuestionResponse.L2, AutoQuestionResponse.L3, AutoQuestionResponse.L4)),
                new AutoQuestion(
                    "First Intaking Position?", List.of(AutoQuestionResponse.RIGHT_FORWARD_CORALSTATION, AutoQuestionResponse.RIGHT_BACK_CORALSTATION, AutoQuestionResponse.LEFT_FORWARD_CORALSTATION, AutoQuestionResponse.LEFT_BACK_CORALSTATION)),
                new AutoQuestion(
                    "Second Scoring Location?",
                    List.of(AutoQuestionResponse.A, AutoQuestionResponse.B, AutoQuestionResponse.C, AutoQuestionResponse.D, AutoQuestionResponse.E, AutoQuestionResponse.F, AutoQuestionResponse.G, AutoQuestionResponse.H, AutoQuestionResponse.I, AutoQuestionResponse.J, AutoQuestionResponse.K, AutoQuestionResponse.L)),
                new AutoQuestion(
                    "Second Scoring Height?", List.of(AutoQuestionResponse.L2, AutoQuestionResponse.L3, AutoQuestionResponse.L4)),
                new AutoQuestion(
                    "Second Intaking Position?", List.of(AutoQuestionResponse.RIGHT_FORWARD_CORALSTATION, AutoQuestionResponse.RIGHT_BACK_CORALSTATION, AutoQuestionResponse.LEFT_FORWARD_CORALSTATION, AutoQuestionResponse.LEFT_BACK_CORALSTATION))
                
            ),        
            autoBuilder.TwoAuto());
        
  }



  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    Container<Translation2d> CoralStationAlignFeedForward = new Container<>();

    // Default command, normal field-relative drive
    Trigger stow = new Trigger(() -> Superstructure.getCurrentState() == SuperState.STOW);

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to nearest coral station's angle when A button is held
    // also go towards it, still allowing for driver translation

    Trigger rightBumper = new Trigger(() -> controller.getRawButton(6));
    // rightBumper.onTrue(superstructure.setWantedSuperStateCommand(SuperState.INTAKE));

    // .whileTrue(
    // new InstantCommand(
    //         () -> {
    //           final double maxVelocity = 1.0;
    //           CoralStationAlignFeedForward.value =
    //               DriveCommands.getLinearVelocityFromJoysticks(
    //                       -controller.getLeftX(), -controller.getLeftY())
    //                   .times(maxVelocity)
    //                   .rotateBy(
    //                       DriverStation.getAlliance().get() == Alliance.Blue
    //                           ? new Rotation2d(Math.PI)
    //                           : new Rotation2d());

    //           coralStationAlignController =
    //               new CoralStationAlignController(
    //                   drive,
    //                   () -> CoralStationAlignFeedForward.value,
    //                   () ->
    //                       RobotState.getInstance()
    //                               .getDistanceToNearestCoralStation(drive.getPose())
    //                           < 1.0);
    //         })
    //     .andThen(
    //         new InstantCommand(
    //                 () -> {
    //                   final double maxVelocity = 1.0;
    //                   CoralStationAlignFeedForward.value =
    //                       DriveCommands.getLinearVelocityFromJoysticks(
    //                               -controller.getLeftY() * 0.5, -controller.getLeftX() * .5)
    //                           .times(maxVelocity)
    //                           .rotateBy(
    //                               DriverStation.getAlliance().get() == Alliance.Blue
    //                                   ? new Rotation2d(Math.PI)
    //                                   : new Rotation2d());
    //                   drive.runVelocity(coralStationAlignController.update().get());
    //                 },
    //                 drive)
    //             .repeatedly())
    //     .andThen(
    //         new ParallelRaceGroup(
    //             new ControllerRumbleCommand(controller, () -> true), new WaitCommand(.4))));

    // auto align to nearest coral station

    rightBumper
        // .onFalse(superstructure.setWantedSuperStateCommand(SuperState.STOW))
        .whileTrue(
        DriveCommands.joystickDriveCoralStation(
            drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    // controller.x().whileTrue(DriveCommands.lineUpToNearestReef(() -> drive.getPose()));

    // controller.x().whileTrue(DriveCommands.lineUpToNearestReef(() -> drive.getPose()));

    Trigger b = new Trigger(() -> controller.getBButton());

    b
        // .onTrue(superstructure.setWantedSuperStateCommand(SuperState.L1))
        //     .onFalse(superstructure.setWantedSuperStateCommand(SuperState.STOW));
        .whileTrue(
        new InstantCommand(
                () -> {
                  reefAlignController =
                      new ReefAlignController(
                          drive,
                          () ->
                              RobotState.getInstance().getDistanceToNearestReef(drive.getPose())
                                  < 1.0,
                          () -> false);
                })
            .andThen(
                new InstantCommand(
                        () -> {
                          drive.runVelocity(reefAlignController.update().get());
                        },
                        drive)
                    .repeatedly())
            .until(() -> reefAlignController.atGoal())
            .andThen(
                new ParallelRaceGroup(
                    new ControllerRumbleCommand(controller, () -> true), new WaitCommand(.4))));

    Trigger a =
        new Trigger(() -> (controller.getAButton() && RobotState.getInstance().isAutoAligning()));

    a
        // .onTrue(
        //         superstructure
        //             .setWantedSuperStateCommand(SuperState.L2)
        //             .andThen(
        //                 new InstantCommand(
        //                     () -> {
        //                       reefAlignController =
        //                           new ReefAlignController(
        //                               drive,
        //                               () ->
        //                                   RobotState.getInstance()
        //                                           .getDistanceToNearestReef(drive.getPose())
        //                                       < 1.0,
        //                               () -> false);
        //                     })))
        // .onFalse(superstructure.setWantedSuperStateCommand(SuperState.STOW))
        // .onFalse(
        //     DriveCommands.joystickDrive(
        //         drive,
        //         () -> -controller.getLeftY(),
        //         () -> -controller.getLeftX(),
        //         () -> -controller.getRightX()))
        .whileTrue(
        new InstantCommand(
                () -> {
                  reefAlignController =
                      new ReefAlignController(
                          drive,
                          () ->
                              RobotState.getInstance().getDistanceToNearestReef(drive.getPose())
                                  < 1.0,
                          () -> false);
                })
            .andThen(
                new InstantCommand(
                        () -> {
                          drive.runVelocity(reefAlignController.update().get());
                        },
                        drive)
                    .repeatedly())
            .until(() -> reefAlignController.atGoal())
            .andThen(
                new ParallelRaceGroup(
                    new ControllerRumbleCommand(controller, () -> true), new WaitCommand(.4))));

    Trigger x =
        new Trigger(() -> (controller.getXButton() && RobotState.getInstance().isAutoAligning()));

    x
        // .onTrue(
        //         superstructure
        //             .setWantedSuperStateCommand(SuperState.L3)
        //             .andThen(
        //                 new InstantCommand(
        //                     () -> {
        //                       reefAlignController =
        //                           new ReefAlignController(
        //                               drive,
        //                               () ->
        //                                   RobotState.getInstance()
        //                                           .getDistanceToNearestReef(drive.getPose())
        //                                       < 1.0,
        //                               () -> false);
        //                     })))
        // .onFalse(superstructure.setWantedSuperStateCommand(SuperState.STOW))
        .onFalse(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()))
        .whileTrue(
            new InstantCommand(
                    () -> {
                      reefAlignController =
                          new ReefAlignController(
                              drive,
                              () ->
                                  RobotState.getInstance().getDistanceToNearestReef(drive.getPose())
                                      < 1.0,
                              () -> false);
                    })
                .andThen(
                    new InstantCommand(
                            () -> {
                              drive.runVelocity(reefAlignController.update().get());
                            },
                            drive)
                        .repeatedly())
                .until(() -> reefAlignController.atGoal())
                .andThen(
                    new ParallelRaceGroup(
                        new ControllerRumbleCommand(controller, () -> true), new WaitCommand(.4))));

    Trigger y =
        new Trigger(() -> (controller.getYButton() && RobotState.getInstance().isAutoAligning()));

    y
        // .onTrue(superstructure.setWantedSuperStateCommand(SuperState.L4))
        //     .onFalse(superstructure.setWantedSuperStateCommand(SuperState.STOW))
        .onFalse(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()))
        .whileTrue(
            new InstantCommand(
                    () -> {
                      reefAlignController =
                          new ReefAlignController(
                              drive,
                              () ->
                                  RobotState.getInstance().getDistanceToNearestReef(drive.getPose())
                                      < 1.0,
                              () -> false);
                    })
                .andThen(
                    new InstantCommand(
                            () -> {
                              drive.runVelocity(reefAlignController.update().get());
                            },
                            drive)
                        .repeatedly())
                .until(() -> reefAlignController.atGoal())
                .andThen(
                    new ParallelRaceGroup(
                        new ControllerRumbleCommand(controller, () -> true), new WaitCommand(.4))));

    Trigger povLeft = new Trigger(() -> controller.getPOV() == 270);
    povLeft.whileTrue(DriveCommands.wheelRadiusCharacterization(drive));

    Trigger povDown = new Trigger(() -> controller.getPOV() == 180);

    povDown.onTrue(
        new InstantCommand(
            () ->
                RobotState.getInstance()
                    .setAddingVision(!RobotState.getInstance().isAddingVision())));

    Trigger povRight = new Trigger(() -> controller.getPOV() == 90);

    // reseting wrist and elevator encoder to their "zero" positions
    // must physically properly be "zeroed" for this to have desired effect
    // povRight.onTrue(
    //     new InstantCommand(
    //             () -> {
    //               elevator.setHeight(0);
    //               wrist.resetPosition(0);
    //             })
    //         .ignoringDisable(true));

    // switch the stick of the reef (on the same face) that is being aligned to.
    // for an easy toggle that can be done while aligning (not letting go of alignment button)
    // repeatedly, safely.
    Trigger leftBumper = new Trigger(() -> controller.getLeftBumperButton());
    leftBumper.onTrue(
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

    // rightTrigger
    //     .onTrue(superstructure.setWantedSuperStateCommand(SuperState.L3L4ALGAE))
    //     .onFalse(superstructure.setWantedSuperStateCommand(SuperState.STOW));

    Trigger leftTrigger = new Trigger(() -> controller.getLeftTriggerAxis() > .5);

    // leftTrigger
    //     .onTrue(superstructure.setWantedSuperStateCommand(SuperState.L2L3ALGAE))
    //     .onFalse(superstructure.setWantedSuperStateCommand(SuperState.STOW));
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

  //   public Superstructure getSuperstructure() {
  //     return superstructure;
  //   }
}
