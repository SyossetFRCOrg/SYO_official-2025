package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.ReefAlignController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.drive.Drive;

/** A factory for creating autonomous programs for a given {@link Auto} */
@SuppressWarnings({"UnusedMethod", "UnusedVariable", "EmptyBlockTag"})
class AutoFactory {
  private static final double AMPBAR_ZERO_DEGREES = 0.0;

  private final DriverStation.Alliance alliance;

  private final RobotContainer robotContainer;
  private final Drive drive;
  private final Superstructure superstructure;
  private boolean trajectoriesLoaded = false;

  private ReefAlignController reefAlignController;

  /**
   * Create a new <code>AutoFactory</code>.
   *
   * @param robotContainer The {@link RobotContainer}
   */
  AutoFactory(
      final DriverStation.Alliance alliance,
      final RobotContainer robotContainer,
      final Drive drive,
      final Superstructure superstructure) {
    this.alliance = alliance;
    this.robotContainer = robotContainer;
    this.drive = drive;
    this.superstructure = superstructure;
    reefAlignController = new ReefAlignController(drive, () -> false, () -> false);
  }

  /* Autonomous program factories
   *
   * Factory methods should be added here for each autonomous program.
   * The factory methods must:
   *   1. Be package-private (i.e. no access modifier)
   *   2. Accept no parameters
   *   3. Return a link Command
   */

  Command createIdleCommand() {
    // return superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.STOPPED);
    return Commands.none();
  }

  Command createLeftStartAllKL() {
    PathPlannerPath firstSegment = loadSegment(Location.LSTART, Location.PREK);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    c.addCommands(resetPose(firstSegment));
    c.addCommands(follow(Location.LSTART, Location.PREK));
    c.addCommands(AutoAlignL4Score());

    c.addCommands(IntakeFollow(Location.K, Location.LEFTCORALSTATION));
    // c.addCommands(waitSeconds(1));

    c.addCommands(StowFollow(Location.LEFTCORALSTATION, Location.PREL));
    c.addCommands(AutoAlignL4Score());

    c.addCommands(IntakeFollow(Location.L, Location.LEFTCORALSTATION));
    // c.addCommands(waitSeconds(1));

    c.addCommands(StowFollow(Location.LEFTCORALSTATION, Location.PREK));
    c.addCommands(AutoAlignL3Score());

    c.addCommands(IntakeFollow(Location.K, Location.LEFTCORALSTATION));
    // c.addCommands(waitSeconds(1));

    c.addCommands(StowFollow(Location.LEFTCORALSTATION, Location.PREL));
    c.addCommands(AutoAlignL3Score());

    c.addCommands(IntakeFollow(Location.L, Location.LEFTCORALSTATION));
    c.addCommands(waitSeconds(1));

    c.addCommands(StowFollow(Location.LEFTCORALSTATION, Location.PREK));
    c.addCommands(AutoAlignL2Score());

    c.addCommands(IntakeFollow(Location.K, Location.LEFTCORALSTATION));
    c.addCommands(waitSeconds(1));

    c.addCommands(StowFollow(Location.LEFTCORALSTATION, Location.PREL));
    c.addCommands(AutoAlignL2Score());
    return c;
  }

  Command createLeftStartAllLK() {
    PathPlannerPath firstSegment = loadSegment(Location.LSTART, Location.PREL);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    c.addCommands(resetPose(firstSegment));

    c.addCommands(follow(Location.LSTART, Location.PREL));
    c.addCommands(AutoAlignL4Score());

    c.addCommands(IntakeFollow(Location.L, Location.LEFTCORALSTATION));
    c.addCommands(waitSeconds(1));

    c.addCommands(StowFollow(Location.LEFTCORALSTATION, Location.PREK));
    c.addCommands(AutoAlignL4Score());

    c.addCommands(IntakeFollow(Location.K, Location.LEFTCORALSTATION));
    c.addCommands(waitSeconds(1));

    c.addCommands(StowFollow(Location.LEFTCORALSTATION, Location.PREL));
    c.addCommands(AutoAlignL3Score());

    c.addCommands(IntakeFollow(Location.L, Location.LEFTCORALSTATION));
    c.addCommands(waitSeconds(1));

    c.addCommands(StowFollow(Location.LEFTCORALSTATION, Location.PREK));
    c.addCommands(AutoAlignL3Score());

    c.addCommands(IntakeFollow(Location.K, Location.LEFTCORALSTATION));
    c.addCommands(waitSeconds(1));

    c.addCommands(StowFollow(Location.LEFTCORALSTATION, Location.PREL));
    c.addCommands(AutoAlignL2Score());

    c.addCommands(IntakeFollow(Location.L, Location.LEFTCORALSTATION));
    c.addCommands(waitSeconds(1));

    c.addCommands(StowFollow(Location.LEFTCORALSTATION, Location.PREK));
    c.addCommands(AutoAlignL2Score());
    return c;
  }

  private Command AutoAlignL4Score() {
    return superstructure
        .setWantedSuperStateCommand(SuperState.L4)
        .andThen(
            new InstantCommand(
                () -> {
                  reefAlignController =
                      new ReefAlignController(
                          drive,
                          () ->
                              RobotState.getInstance().getDistanceToNearestReef(drive.getPose())
                                  < 1.0,
                          () -> false);
                }))
        .andThen(
            new InstantCommand(
                    () -> {
                      drive.runVelocity(reefAlignController.update().get());
                    },
                    drive)
                .repeatedly()
                .until(
                    () ->
                        reefAlignController.atGoal()
                            && Superstructure.getCurrentState() == SuperState.L4))
        .andThen(waitSeconds(1));
  }

  private Command AutoAlignL3Score() {
    return superstructure
        .setWantedSuperStateCommand(SuperState.L3)
        .andThen(
            new InstantCommand(
                () -> {
                  reefAlignController =
                      new ReefAlignController(
                          drive,
                          () ->
                              RobotState.getInstance().getDistanceToNearestReef(drive.getPose())
                                  < 1.0,
                          () -> false);
                }))
        .andThen(
            new InstantCommand(
                    () -> {
                      drive.runVelocity(reefAlignController.update().get());
                    },
                    drive)
                .repeatedly()
                .until(
                    () ->
                        reefAlignController.atGoal()
                            && Superstructure.getCurrentState() == SuperState.L3))
        .andThen(waitSeconds(1));
  }

  private Command AutoAlignL2Score() {
    return superstructure
        .setWantedSuperStateCommand(SuperState.L2)
        .andThen(
            new InstantCommand(
                () -> {
                  reefAlignController =
                      new ReefAlignController(
                          drive,
                          () ->
                              RobotState.getInstance().getDistanceToNearestReef(drive.getPose())
                                  < 1.0,
                          () -> false);
                }))
        .andThen(
            new InstantCommand(
                    () -> {
                      drive.runVelocity(reefAlignController.update().get());
                    },
                    drive)
                .repeatedly()
                .until(
                    () ->
                        reefAlignController.atGoal()
                            && Superstructure.getCurrentState() == SuperState.L2))
        .andThen(waitSeconds(1));
  }

  // Auto init helpers
  private Command resetPose(final PathPlannerPath segment) {
    return runOnce(
        () -> {
          var correctedTraj =
              segment.generateTrajectory(new ChassisSpeeds(), new Rotation2d(), null);
          Pose2d pose = correctedTraj.getInitialPose();
          // Pose2d pose = segment.getPreviewStartingHolonomicPose();
          // //getpreviewstartingholonomicpose didn't work
          // getStartingDifferentialPose worked!!!

          drive.setPose(pose);
        });
  }

  // Auto init helpers
  private Command resetPose(final Pose2d pose) {
    return runOnce(
        () -> {
          // Pose2d pose = segment.getPreviewStartingHolonomicPose();
          // //getpreviewstartingholonomicpose didn't work
          // getStartingDifferentialPose worked!!!

          drive.setPose(pose);
        });
  }

  private Command StowFollow(final Location start, final Location end) {
    return superstructure
        .setWantedSuperStateCommand(SuperState.STOW)
        .alongWith(follow(loadSegment(start, end)));
  }

  private Command IntakeFollow(final Location start, final Location end) {
    return (superstructure
        .setWantedSuperStateCommand(SuperState.INTAKE)
        .alongWith(follow(loadSegment(start, end))));
    // time for HP to throw in the coral
  }

  // Path following
  private Command follow(final Location start, final Location end) {
    return follow(loadSegment(start, end));
  }

  // Path following
  private Command follow(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
  }

  private void preloadTrajectoryClass(PathPlannerPath firstSegment) {
    // This is done because Java loads classes lazily. Calling this here loads the trajectory class
    // which is used to follow paths and saves user code ms loop time at the start of auto.
    if (!trajectoriesLoaded) {
      trajectoriesLoaded = true;
      var trajectory =
          new PathPlannerTrajectory(
              firstSegment, drive.getChassisSpeeds(), drive.getPose().getRotation(), null);
    }
  }

  // Load paths
  private PathPlannerPath loadSegment(final Location start, final Location end) {
    var name = "%S_TO_%S".formatted(start, end);
    PathPlannerPath path;

    try {
      path = PathPlannerPath.fromChoreoTrajectory("%S%S".formatted("BLUE", name));
    } catch (Exception e) {
      e.printStackTrace();
      path = null;
    }

    path.preventFlipping = true;

    // return new AutoSegment(start, end, name, path);
    return path;
  }
}
