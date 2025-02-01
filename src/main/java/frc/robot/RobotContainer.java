package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ControllerRumbleCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ReefAlignController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;

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

  // Controller
  private final XboxController controller = new XboxController(0);

  //   private ReefAlignController autoAlignController;
  private ReefAlignController reefAlignController;

  //   // Dashboard inputs
  //   private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        elevator = new Elevator(new ElevatorIOTalonFX());
        intake = new Intake(new IntakeIOSparkMax());
        break;

        //   case SIM:
        //     // Sim robot, instantiate physics sim IO implementations
        //     drive =
        //         new Drive(
        //             new GyroIO() {},
        //             new ModuleIOSim(TunerConstants.FrontLeft),
        //             new ModuleIOSim(TunerConstants.FrontRight),
        //             new ModuleIOSim(TunerConstants.BackLeft),
        //             new ModuleIOSim(TunerConstants.BackRight));
        //     break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        intake = new Intake(new IntakeIO() {});
        break;
    }
    // switch (Constants.currentMode) {
    //   case REAL:
    //     // Real robot, instantiate hardware IO implementations
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));

    // vision =
    //     new Vision(
    //         demoDrive::addVisionMeasurement,
    //         new VisionIOPhotonVision(camera0Name, robotToCamera0),
    //         new VisionIOPhotonVision(camera1Name, robotToCamera1));
    // break;

    //   case SIM:
    //     // Sim robot, instantiate physics sim IO implementations
    //     vision =
    //         new Vision(
    //             drive::addVisionMeasurement,
    //             new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
    //             new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
    //     break;

    //   default:
    //     // Replayed robot, disable IO implementations
    //     // (Use same number of dummy implementations as the real robot)
    //     vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
    //     break;
    // }

    // Set up auto routines
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
    reefAlignController = new ReefAlignController(drive, () -> false, () -> false);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // controller.setRumble(null, 0);
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to nearest coral station's angle when A button is held
    Trigger a = new Trigger(() -> controller.getAButton());
    a.whileTrue(
        DriveCommands.joystickDriveCoralStation(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> drive.getPose().getY()));

    // controller.x().whileTrue(DriveCommands.lineUpToNearestReef(() -> drive.getPose()));

    // controller.x().whileTrue(DriveCommands.lineUpToNearestReef(() -> drive.getPose()));
    Trigger y = new Trigger(() -> controller.getYButton());
    y.whileTrue(
        new InstantCommand(
                () -> {
                  reefAlignController =
                      new ReefAlignController(
                          drive,
                          () ->
                              RobotState.getInstance().getDistanceToNearestReef(drive.getPose())
                                  < 1.5,
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
            .andThen(new ControllerRumbleCommand(controller, () -> true)));

    Trigger leftBumper = new Trigger(() -> controller.getLeftBumperButton());
    leftBumper.onTrue(
        new InstantCommand( () -> reefAlignController =
    new ReefAlignController(
        drive,
        () ->
            RobotState.getInstance().getDistanceToNearestReef(drive.getPose())
                < 1.5,
        () -> true)));

    // Reset gyro to 0° when B button is pressed
    Trigger b = new Trigger(() -> controller.getBButton());
    b.onTrue(
        Commands.runOnce(
                () ->
                    drive.setPose(
                        new Pose2d(
                            3.442369222640991,
                            5.234981060028076,
                            Rotation2d.fromRadians(-1.0486071798869254))),
                drive)
            .ignoringDisable(true));
  }

  //   /**
  //    * Use this to pass the autonomous command to the main {@link Robot} class.
  //    *
  //    * @return the command to run in autonomous
  //    */
  //   public Command getAutonomousCommand() {
  //     return autoChooser.get();
  //   }
  
  public ReefAlignController getReefAlignController(){
    return reefAlignController;
  }
}
