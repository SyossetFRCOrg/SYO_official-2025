package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.camera2Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ControllerRumbleCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ReefAlignController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOTalonFX;
import frc.robot.util.AllianceFlipUtil;
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
  private SwerveDriveSimulation driveSimulation = null;
  private final Intake intake;
  private final Wrist wrist;
  private final Superstructure superstructure;

  private final Climber climber;

  //   private final LEDs LEDs;

  // Controller
  private final XboxController controller = new XboxController(0);
  private final XboxController buttonboard = new XboxController(1);

  //private final UsbCamera climbCam;

  //   private final HttpCamera climberCamera;

  //   private final AutoSelector autoSelector = new AutoSelector("Auto");

  //   private ReefAlignController autoAlignController;
  private ReefAlignController reefAlignController;

  //   // Dashboard inputs
  //   private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode){
        case REAL ->
        {
    drive =
        new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
            new ModuleIOTalonFXReal(TunerConstants.FrontRight),
            new ModuleIOTalonFXReal(TunerConstants.BackLeft),
            new ModuleIOTalonFXReal(TunerConstants.BackRight),
            (pose) -> {});

    elevator = new Elevator(new ElevatorIOTalonFX());
    wrist = new Wrist(new WristIOTalonFX());
    intake = new Intake(new IntakeIOTalonFX());

    // LEDs = new LEDs();

    vision =
        new Vision(
            drive::addVisionMeasurement,
            drive,
            new VisionIOLimelight(camera0Name, drive::getRotation),
            new VisionIOLimelight(camera1Name, drive::getRotation),
            new VisionIOLimelight(camera2Name, drive::getRotation));

    reefAlignController = new ReefAlignController(drive, () -> false, () -> false);

    climber = new Climber(new ClimberIOTalonFX());
    // climbCam = CameraServer.startAutomaticCapture();
    // climbCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    // climbCam.setResolution(80, 60);
    // Shuffleboard.getTab("Match")
    // .add(new HttpCamera("ClimberCam", "http://roborio-9016-frc.local:1181/?action=stream"))
    // .withWidget(BuiltInWidgets.kCameraStream)
    // .withSize(4, 3)
    // .withPosition(4, 3);
    }
        case SIM -> 
        {
          // Sim robot, instantiate physics sim IO implementations

                driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackRight, driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);
                vision = new Vision(
                        drive::addVisionMeasurement,
                        drive,
                        new VisionIOPhotonVisionSim(
                                camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));


        }
        
        default ->
        {
                 drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {}, 
                        (pose) -> {});
                elevator = new Elevator(new ElevatorIO() {});
                wrist = new Wrist(new WristIO() {});
                vision = new Vision(drive::addVisionMeasurement,drive, new VisionIO() {}, new VisionIO() {});
        }
    }
    superstructure = new Superstructure(drive, elevator, wrist, this);
        
    // configureAutos();

    // Configure the button bindings
    configureButtonBindings();

    // climberCamera =
    //     new HttpCamera("ClimberCamera", "http://roborio-9016-frc.local:1181/?action=stream");
    // climberCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    // Shuffleboard.getTab("Match")
    //     .add(climberCamera)
    //     .withWidget(BuiltInWidgets.kCameraStream)
    //     .withSize(2, 2)
    //     .withPosition(4, 0);


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // kinda stupid but it works
    Container<Translation2d> AAFFSupplier = new Container<>();

    double tempSpeed = 0.40;

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * tempSpeed,
            () -> -controller.getLeftX() * tempSpeed,
            () -> -controller.getRightX() * tempSpeed));

    // Lock to nearest coral station's angle when A button is held
    // also go towards it, still allowing for driver translation

    Trigger AArightBumper =
        new Trigger(
            () ->
                controller.getRawButton(6)
                    && RobotState.getInstance().isIntakeAutoAiming()
                    && !controller.getLeftBumperButton());
    AArightBumper.onTrue(superstructure.setWantedSuperStateCommand(SuperState.INTAKE));

    AArightBumper.onFalse(superstructure.setWantedSuperStateCommand(SuperState.STOW));

    AArightBumper.whileTrue(
        DriveCommands.joystickDriveCoralStation(
            drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    Trigger noAArightBumper =
        new Trigger(
            () ->
                controller.getRawButton(6)
                    && !RobotState.getInstance().isIntakeAutoAiming()
                    && !controller.getLeftBumperButton());
    noAArightBumper.onTrue(superstructure.setWantedSuperStateCommand(SuperState.INTAKE));
    noAArightBumper.onFalse(superstructure.setWantedSuperStateCommand(SuperState.STOW));

    Trigger AArightBumperLowIntake =
        new Trigger(
            () ->
                controller.getRawButton(6)
                    && RobotState.getInstance().isIntakeAutoAiming()
                    && controller.getLeftBumperButton());
    AArightBumperLowIntake.onTrue(superstructure.setWantedSuperStateCommand(SuperState.INTAKELOW));

    AArightBumperLowIntake.onFalse(superstructure.setWantedSuperStateCommand(SuperState.STOW));

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
        superstructure.setWantedSuperStateCommand(SuperState.INTAKELOW));
    noAArightBumperLowIntake.onFalse(superstructure.setWantedSuperStateCommand(SuperState.STOW));

    Trigger noAAL1 = new Trigger(() -> controller.getBButton());

    noAAL1
        .onTrue(superstructure.setWantedSuperStateCommand(SuperState.L1PREPARE))
        .onFalse(
            new WaitCommand(1)
                .deadlineFor(superstructure.setWantedSuperStateCommand(SuperState.L1))
                .andThen(superstructure.setWantedSuperStateCommand(SuperState.STOW)));

    Trigger AAL2 =
        new Trigger(() -> controller.getAButton() && RobotState.getInstance().isReefAutoAligning());

    AAL2.onTrue(
            superstructure
                .setWantedSuperStateCommand(SuperState.L2PREPARE)
                .andThen(
                    new InstantCommand(
                        () -> {
                          reefAlignController =
                              new ReefAlignController(
                                  drive,
                                  () -> AAFFSupplier.value,
                                  () ->
                                      RobotState.getInstance()
                                              .getDistanceToNearestReef(drive.getPose())
                                          < 1.0,
                                  () -> false);
                        })))
        .onFalse(
            new WaitCommand(.5)
                .deadlineFor(superstructure.setWantedSuperStateCommand(SuperState.L2))
                .andThen(superstructure.setWantedSuperStateCommand(SuperState.STOW)))
        .onFalse(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()))
        .whileTrue(
            new InstantCommand(
                    () -> {
                      AAFFSupplier.value =
                          DriveCommands.getLinearVelocityFromJoysticks(
                                  -controller.getLeftY(), -controller.getLeftX())
                              .times(RobotState.getInstance().getModuleLimits().maxDriveVelocity())
                              .rotateBy(
                                  AllianceFlipUtil.shouldFlip()
                                      ? new Rotation2d(Math.PI)
                                      : new Rotation2d())
                              .plus(
                                  DriveCommands.getLinearVelocityFromJoysticks(
                                          -controller.getLeftY(), -controller.getLeftX())
                                      .times(
                                          RobotState.getInstance()
                                              .getModuleLimits()
                                              .maxDriveVelocity())
                                      .rotateBy(
                                          AllianceFlipUtil.shouldFlip()
                                              ? new Rotation2d(Math.PI)
                                              : new Rotation2d())
                                      .times(0.02));

                      drive.runVelocity(reefAlignController.update().get());
                    },
                    drive)
                .repeatedly()
                .until(() -> reefAlignController.atGoal())
                .andThen(
                    new ParallelRaceGroup(
                        new ControllerRumbleCommand(controller, () -> true), new WaitCommand(.2))));

    Trigger noAAL2 =
        new Trigger(
            () -> controller.getAButton() && !RobotState.getInstance().isReefAutoAligning());

    noAAL2
        .onTrue(superstructure.setWantedSuperStateCommand(SuperState.L2PREPARE))
        .onFalse(
            new WaitCommand(.2)
                .deadlineFor(superstructure.setWantedSuperStateCommand(SuperState.L2))
                .andThen(superstructure.setWantedSuperStateCommand(SuperState.STOW)));

    Trigger AAimL2 =
        new Trigger(
            () ->
                (controller.getAButton()
                    && !RobotState.getInstance().isReefAutoAligning()
                    && RobotState.getInstance().isReefAutoAiming()));

    AAimL2.onTrue(superstructure.setWantedSuperStateCommand(SuperState.L2PREPARE))
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> RobotState.getInstance().getNearestReefPose(drive.getPose()).getRotation()))
        .onFalse(
            new WaitCommand(.2)
                .deadlineFor(superstructure.setWantedSuperStateCommand(SuperState.L2))
                .andThen(superstructure.setWantedSuperStateCommand(SuperState.STOW)));

    Trigger AAL3 =
        new Trigger(
            () -> (controller.getXButton() && RobotState.getInstance().isReefAutoAligning()));

    AAL3.onTrue(
            superstructure
                .setWantedSuperStateCommand(SuperState.L3PREPARE)
                .andThen(
                    new InstantCommand(
                        () -> {
                          reefAlignController =
                              new ReefAlignController(
                                  drive,
                                  () -> AAFFSupplier.value,
                                  () ->
                                      RobotState.getInstance()
                                              .getDistanceToNearestReef(drive.getPose())
                                          < 1.0,
                                  () -> false);
                        })))
        .onFalse(
            new WaitCommand(.5)
                .deadlineFor(superstructure.setWantedSuperStateCommand(SuperState.L3))
                .andThen(superstructure.setWantedSuperStateCommand(SuperState.STOW)))
        .onFalse(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()))
        .whileTrue(
            new InstantCommand(
                    () -> {
                      AAFFSupplier.value =
                          DriveCommands.getLinearVelocityFromJoysticks(
                                  -controller.getLeftY(), -controller.getLeftX())
                              .times(RobotState.getInstance().getModuleLimits().maxDriveVelocity())
                              .rotateBy(
                                  AllianceFlipUtil.shouldFlip()
                                      ? new Rotation2d(Math.PI)
                                      : new Rotation2d())
                              .plus(
                                  DriveCommands.getLinearVelocityFromJoysticks(
                                          -controller.getLeftY(), -controller.getLeftX())
                                      .times(
                                          RobotState.getInstance()
                                              .getModuleLimits()
                                              .maxDriveVelocity())
                                      .rotateBy(
                                          AllianceFlipUtil.shouldFlip()
                                              ? new Rotation2d(Math.PI)
                                              : new Rotation2d())
                                      .times(0.02));
                      drive.runVelocity(reefAlignController.update().get());
                    },
                    drive)
                .repeatedly()
                .until(() -> reefAlignController.atGoal())
                .andThen(
                    new ParallelRaceGroup(
                        new ControllerRumbleCommand(controller, () -> true), new WaitCommand(.2))));

    Trigger noAAL3 =
        new Trigger(
            () -> controller.getXButton() && !RobotState.getInstance().isReefAutoAligning());

    noAAL3
        .onTrue(superstructure.setWantedSuperStateCommand(SuperState.L3PREPARE))
        .onFalse(
            new WaitCommand(.2)
                .deadlineFor(superstructure.setWantedSuperStateCommand(SuperState.L3))
                .andThen(superstructure.setWantedSuperStateCommand(SuperState.STOW)));

    Trigger AAimL3 =
        new Trigger(
            () ->
                (controller.getXButton()
                    && !RobotState.getInstance().isReefAutoAligning()
                    && RobotState.getInstance().isReefAutoAiming()));

    AAimL3.onTrue(superstructure.setWantedSuperStateCommand(SuperState.L3PREPARE))
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> RobotState.getInstance().getNearestReefPose(drive.getPose()).getRotation()))
        .onFalse(
            new WaitCommand(.2)
                .deadlineFor(superstructure.setWantedSuperStateCommand(SuperState.L3))
                .andThen(superstructure.setWantedSuperStateCommand(SuperState.STOW)));

    Trigger AAL4 =
        new Trigger(
            () -> (controller.getYButton() && RobotState.getInstance().isReefAutoAligning()));

    AAL4.onTrue(
            superstructure
                .setWantedSuperStateCommand(SuperState.L4PREPARE)
                .andThen(
                    new InstantCommand(
                        () -> {
                          reefAlignController =
                              new ReefAlignController(
                                  drive,
                                  () -> AAFFSupplier.value,
                                  () ->
                                      RobotState.getInstance()
                                              .getDistanceToNearestReef(drive.getPose())
                                          < 1.0,
                                  () -> false);
                        })))
        .onFalse(
            new WaitCommand(.5)
                .deadlineFor(superstructure.setWantedSuperStateCommand(SuperState.L4))
                .andThen(superstructure.setWantedSuperStateCommand(SuperState.STOW)))
        .onFalse(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()))
        .whileTrue(
            new InstantCommand(
                    () -> {
                      AAFFSupplier.value =
                          DriveCommands.getLinearVelocityFromJoysticks(
                                  -controller.getLeftY(), -controller.getLeftX())
                              .times(RobotState.getInstance().getModuleLimits().maxDriveVelocity())
                              .rotateBy(
                                  AllianceFlipUtil.shouldFlip()
                                      ? new Rotation2d(Math.PI)
                                      : new Rotation2d())
                              .plus(
                                  DriveCommands.getLinearVelocityFromJoysticks(
                                          -controller.getLeftY(), -controller.getLeftX())
                                      .times(
                                          RobotState.getInstance()
                                              .getModuleLimits()
                                              .maxDriveVelocity())
                                      .rotateBy(
                                          AllianceFlipUtil.shouldFlip()
                                              ? new Rotation2d(Math.PI)
                                              : new Rotation2d())
                                      .times(0.02));
                      drive.runVelocity(reefAlignController.update().get());
                    },
                    drive)
                .repeatedly()
                .until(() -> reefAlignController.atGoal())
                .andThen(
                    new ParallelRaceGroup(
                        new ControllerRumbleCommand(controller, () -> true), new WaitCommand(.2))));

    Trigger noAAL4 =
        new Trigger(
            () ->
                controller.getYButton()
                    && !RobotState.getInstance().isReefAutoAligning()
                    && !RobotState.getInstance().isReefAutoAiming());

    noAAL4
        .onTrue(superstructure.setWantedSuperStateCommand(SuperState.L4PREPARE))
        .onFalse(
            new WaitCommand(.2)
                .deadlineFor(superstructure.setWantedSuperStateCommand(SuperState.L4))
                .andThen(superstructure.setWantedSuperStateCommand(SuperState.STOW)));

    Trigger AAimL4 =
        new Trigger(
            () ->
                (controller.getYButton()
                    && !RobotState.getInstance().isReefAutoAligning()
                    && RobotState.getInstance().isReefAutoAiming()));

    AAimL4.onTrue(superstructure.setWantedSuperStateCommand(SuperState.L4PREPARE))
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> RobotState.getInstance().getNearestReefPose(drive.getPose()).getRotation()))
        .onFalse(
            new WaitCommand(.2)
                .deadlineFor(superstructure.setWantedSuperStateCommand(SuperState.L4))
                .andThen(superstructure.setWantedSuperStateCommand(SuperState.STOW)));

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

    Trigger elevatorUpManual =
        new Trigger(() -> buttonboard.getRawButton(6) && !(buttonboard.getRawAxis(3) > .5));
    elevatorUpManual.whileTrue(
        new InstantCommand(
                () -> {
                  elevator.setHeight(elevator.getHeight() - .2);
                })
            .repeatedly()
            .ignoringDisable(true));

    Trigger elevatorDownManual =
        new Trigger(() -> (buttonboard.getRawAxis(3) > .5) && !buttonboard.getRawButton(6));
    elevatorDownManual.whileTrue(
        new InstantCommand(
                () -> {
                  elevator.setHeight(elevator.getHeight() + .2);
                })
            .repeatedly()
            .ignoringDisable(true));

    Trigger limitSwitching =
        new Trigger(() -> (buttonboard.getRawAxis(3) > .5) && buttonboard.getRawButton(6));

    limitSwitching.onTrue(
        new InstantCommand(
            () -> {
              RobotState.getInstance()
                  .setLimitSwitching(!RobotState.getInstance().isLimitSwitching());
            }));

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
        .onTrue(superstructure.setWantedSuperStateCommand(SuperState.L3L4ALGAE))
        .onFalse(superstructure.setWantedSuperStateCommand(SuperState.STOW));

    Trigger leftTrigger = new Trigger(() -> controller.getLeftTriggerAxis() > .5);

    leftTrigger
        .onTrue(superstructure.setWantedSuperStateCommand(SuperState.L2L3ALGAE))
        .onFalse(superstructure.setWantedSuperStateCommand(SuperState.STOW));

    Trigger climberUpTrigger = new Trigger(() -> controller.getPOV() == 0);
    climberUpTrigger
        .onTrue(
            // moving elevator up during climb, we want the robot to lean backwards
            // so the chain doesn't touch the elevator (hopefully)
            superstructure
                .setWantedSuperStateCommand(SuperState.L1PREPARE)
                .alongWith(climber.setMotorVoltage(-6)))
        .onFalse(climber.setMotorVoltage(0));

    Trigger climberDownTrigger = new Trigger(() -> controller.getPOV() == 180);
    climberDownTrigger
        .onTrue(
            superstructure
                .setWantedSuperStateCommand(SuperState.L1PREPARE)
                .alongWith(climber.setMotorVoltage(6)))
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
  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
    }
    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }

}
