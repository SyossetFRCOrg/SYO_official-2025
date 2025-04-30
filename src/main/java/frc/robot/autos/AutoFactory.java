package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.AutonReefAlignController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

/** A factory for creating autonomous programs for a given {@link Auto} */
@SuppressWarnings({"UnusedMethod", "UnusedVariable", "EmptyBlockTag"})
class AutoFactory {
  private static final double AMPBAR_ZERO_DEGREES = 0.0;

  private final DriverStation.Alliance alliance;

  private final RobotContainer robotContainer;
  private final Drive drive;
  private final Superstructure superstructure;
  private boolean trajectoriesLoaded = false;

  private AutonReefAlignController autonreefAlignController;

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
    autonreefAlignController = new AutonReefAlignController(drive, () -> false, new Pose2d());
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

  // Left Side Autons (hopefully)
  Command FarLeftStartForwardCoralStationHIJ() {
    PathPlannerPath firstSegment = loadSegment(Location.FARLEFTSTART, Location.PREH);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.H, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREI));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREI)));

    c.addCommands(IntakeFollow(Location.I, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREJ));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREJ)));

    return c;
  }

  Command FarLeftStartForwardCoralStationIJK() {
    PathPlannerPath firstSegment = loadSegment(Location.FARLEFTSTART, Location.PREI);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.I, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREJ));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREJ)));

    c.addCommands(IntakeFollow(Location.J, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREK));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREK)));

    return c;
  }

  Command FarLeftStartForwardCoralStationJKL() {
    PathPlannerPath firstSegment = loadSegment(Location.FARLEFTSTART, Location.PREJ);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.J, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREK));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREK)));

    c.addCommands(IntakeFollow(Location.K, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREL));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREL)));

    return c;
  }

  Command FarLeftStartForwardCoralStationJIH() {
    PathPlannerPath firstSegment = loadSegment(Location.FARLEFTSTART, Location.PREJ);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.J, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREH));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREH)));

    c.addCommands(IntakeFollow(Location.H, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREI));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREI)));

    return c;
  }

  Command FarLeftStartForwardCoralStationKJI() {
    PathPlannerPath firstSegment = loadSegment(Location.FARLEFTSTART, Location.PREK);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.K, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREJ));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREJ)));

    c.addCommands(IntakeFollow(Location.J, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREI));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREI)));

    // third one L2 for now

    return c;
  }

  Command FarLeftStartForwardCoralStationHGI() {
    PathPlannerPath firstSegment = loadSegment(Location.FARLEFTSTART, Location.PREH);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.H, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREG));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREG)));

    c.addCommands(IntakeFollow(Location.G, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREI));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREI)));

    return c;
  }

  Command MidLeftStartForwardCoralStationHIJ() {
    PathPlannerPath firstSegment = loadSegment(Location.MIDLEFTSTART, Location.PREH);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.H, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREI));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREI)));

    c.addCommands(IntakeFollow(Location.I, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREJ));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREJ)));

    return c;
  }

  Command MidLeftStartForwardCoralStationHGI() {
    PathPlannerPath firstSegment = loadSegment(Location.MIDLEFTSTART, Location.PREH);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.H, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREG));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREG)));

    c.addCommands(IntakeFollow(Location.G, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREI));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREI)));

    return c;
  }

  Command MidLeftStartForwardCoralStationGHI() {
    PathPlannerPath firstSegment = loadSegment(Location.MIDLEFTSTART, Location.PREG);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.G, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREH));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREH)));

    c.addCommands(IntakeFollow(Location.H, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREI));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREI)));

    return c;
  }

  Command MidLeftStartForwardCoralStationIJK() {
    PathPlannerPath firstSegment = loadSegment(Location.MIDLEFTSTART, Location.PREI);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.I, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREJ));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREJ)));

    c.addCommands(IntakeFollow(Location.J, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREK));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREK)));

    return c;
  }

  Command MidLeftStartForwardCoralStationJKL() {
    PathPlannerPath firstSegment = loadSegment(Location.MIDLEFTSTART, Location.PREJ);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.J, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREK));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREK)));

    c.addCommands(IntakeFollow(Location.K, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREL));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREL)));

    return c;
  }

  Command MidLeftStartForwardCoralStationJIH() {
    PathPlannerPath firstSegment = loadSegment(Location.MIDLEFTSTART, Location.PREJ);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.J, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREH));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREH)));

    c.addCommands(IntakeFollow(Location.H, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREI));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREI)));

    return c;
  }

  Command MidLeftStartForwardCoralStationKJI() {
    PathPlannerPath firstSegment = loadSegment(Location.MIDLEFTSTART, Location.PREK);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.K, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREJ));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREJ)));

    c.addCommands(IntakeFollow(Location.J, Location.LEFTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTFORWARDCORALSTATION, Location.PREI));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTFORWARDCORALSTATION, Location.PREI)));

    return c;
  }

  Command FarLeftStartBackCoralStationKLA() {
    PathPlannerPath firstSegment = loadSegment(Location.FARLEFTSTART, Location.PREK);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.K, Location.LEFTBACKCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTBACKCORALSTATION, Location.PREL));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTBACKCORALSTATION, Location.PREL)));

    c.addCommands(IntakeFollow(Location.L, Location.LEFTBACKCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTBACKCORALSTATION, Location.PREA));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTBACKCORALSTATION, Location.PREA)));

    return c;
  }

  Command FarLeftStartBackCoralStationLAB() {
    PathPlannerPath firstSegment = loadSegment(Location.FARLEFTSTART, Location.PREL);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.L, Location.LEFTBACKCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTBACKCORALSTATION, Location.PREA));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTBACKCORALSTATION, Location.PREA)));

    c.addCommands(IntakeFollow(Location.A, Location.LEFTBACKCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.LEFTBACKCORALSTATION, Location.PREB));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.LEFTBACKCORALSTATION, Location.PREB)));

    return c;
  }

  /////////// right side autons (hopefully)
  ///
  ///
  ///
  ///
  ///
  ///
  ///
  ///
  ///
  ///

  Command FarRightStartForwardCoralStationGFE() {
    PathPlannerPath firstSegment = loadSegment(Location.FARRIGHTSTART, Location.PREG);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.G, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREF));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREF)));

    c.addCommands(IntakeFollow(Location.F, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREE));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREE)));

    return c;
  }

  Command FarRightStartForwardCoralStationFED() {
    PathPlannerPath firstSegment = loadSegment(Location.FARRIGHTSTART, Location.PREF);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.F, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREE));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREE)));

    c.addCommands(IntakeFollow(Location.E, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PRED));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PRED)));

    return c;
  }

  Command FarRightStartForwardCoralStationEDC() {
    PathPlannerPath firstSegment = loadSegment(Location.FARRIGHTSTART, Location.PREE);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.E, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PRED));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PRED)));

    c.addCommands(IntakeFollow(Location.D, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREC));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREC)));

    return c;
  }

  Command FarRightStartForwardCoralStationEFG() {
    PathPlannerPath firstSegment = loadSegment(Location.FARRIGHTSTART, Location.PREE);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.E, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREF));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREF)));

    c.addCommands(IntakeFollow(Location.F, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREG));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREG)));

    return c;
  }

  Command FarRightStartForwardCoralStationDEF() {
    PathPlannerPath firstSegment = loadSegment(Location.FARRIGHTSTART, Location.PRED);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.D, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREE));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREE)));

    c.addCommands(IntakeFollow(Location.E, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREF));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREF)));

    return c;
  }

  Command FarRightStartForwardCoralStationGHF() {
    PathPlannerPath firstSegment = loadSegment(Location.FARRIGHTSTART, Location.PREG);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.G, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREH));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREH)));

    c.addCommands(IntakeFollow(Location.H, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREF));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREF)));

    return c;
  }

  Command MidRightStartForwardCoralStationGFE() {
    PathPlannerPath firstSegment = loadSegment(Location.MIDRIGHTSTART, Location.PREG);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.G, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREF));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREF)));

    c.addCommands(IntakeFollow(Location.F, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREE));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREE)));

    return c;
  }

  Command MidRightStartForwardCoralStationGHF() {
    PathPlannerPath firstSegment = loadSegment(Location.MIDRIGHTSTART, Location.PREG);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.G, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREH));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREH)));

    c.addCommands(IntakeFollow(Location.G, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREF));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREF)));

    return c;
  }

  Command MidRightStartForwardCoralStationHGF() {
    PathPlannerPath firstSegment = loadSegment(Location.MIDLEFTSTART, Location.PREH);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.H, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREG));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREG)));

    c.addCommands(IntakeFollow(Location.G, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREF));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREF)));

    return c;
  }

  Command MidRightStartForwardCoralStationFED() {
    PathPlannerPath firstSegment = loadSegment(Location.MIDRIGHTSTART, Location.PREF);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.F, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREE));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREE)));

    c.addCommands(IntakeFollow(Location.E, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PRED));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PRED)));

    return c;
  }

  Command MidRightStartForwardCoralStationEDC() {
    PathPlannerPath firstSegment = loadSegment(Location.MIDRIGHTSTART, Location.PREE);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.E, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PRED));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PRED)));

    c.addCommands(IntakeFollow(Location.D, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREC));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREC)));

    return c;
  }

  Command MidRightStartForwardCoralStationEFG() {
    PathPlannerPath firstSegment = loadSegment(Location.MIDRIGHTSTART, Location.PREE);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.E, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREF));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREF)));

    c.addCommands(IntakeFollow(Location.F, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREG));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREG)));

    return c;
  }

  Command MidRightStartForwardCoralStationDEF() {
    PathPlannerPath firstSegment = loadSegment(Location.MIDRIGHTSTART, Location.PRED);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.D, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREE));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREE)));

    c.addCommands(IntakeFollow(Location.E, Location.RIGHTFORWARDCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTFORWARDCORALSTATION, Location.PREF));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTFORWARDCORALSTATION, Location.PREF)));

    return c;
  }

  Command FarRightStartBackCoralStationDCB() {
    PathPlannerPath firstSegment = loadSegment(Location.FARRIGHTSTART, Location.PRED);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.D, Location.RIGHTBACKCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTBACKCORALSTATION, Location.PREC));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTBACKCORALSTATION, Location.PREC)));

    c.addCommands(IntakeFollow(Location.C, Location.RIGHTBACKCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTBACKCORALSTATION, Location.PREB));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTBACKCORALSTATION, Location.PREB)));

    return c;
  }

  Command FarRightStartBackCoralStationCBA() {
    PathPlannerPath firstSegment = loadSegment(Location.FARRIGHTSTART, Location.PREC);

    preloadTrajectoryClass(firstSegment);
    SequentialCommandGroup c = new SequentialCommandGroup();
    // c.addCommands(resetPose(firstSegment));//
    c.addCommands(StowFollow(firstSegment));
    c.addCommands(AutoAlignL4Score(firstSegment));

    c.addCommands(IntakeFollow(Location.C, Location.RIGHTBACKCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTBACKCORALSTATION, Location.PREB));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTBACKCORALSTATION, Location.PREB)));

    c.addCommands(IntakeFollow(Location.B, Location.RIGHTBACKCORALSTATION));
    c.addCommands(waitSeconds(.04));

    c.addCommands(L3PrepareFollow(Location.RIGHTBACKCORALSTATION, Location.PREA));
    c.addCommands(AutoAlignL4Score(loadSegment(Location.RIGHTBACKCORALSTATION, Location.PREA)));

    return c;
  }

  ///////////////////////// util functions

  private Command AutoAlignL4Score(PathPlannerPath segment) {
    return superstructure
        .setWantedSuperStateCommand(SuperState.L4PREPARE)
        .andThen(
            new InstantCommand(
                () -> {
                  autonreefAlignController =
                      new AutonReefAlignController(
                          drive,
                          () ->
                              RobotState.getInstance().getDistanceToNearestReef(drive.getPose())
                                  < 1.0,
                          AllianceFlipUtil.apply(
                              RobotState.getInstance()
                                  .getNearestReefPose(
                                      segment.getIdealTrajectory(null).get().getEndState().pose)));
                }))
        .andThen(
            new InstantCommand(
                    () -> {
                      drive.runVelocity(autonreefAlignController.update().get());
                    },
                    drive)
                .repeatedly()
                .until(() -> autonreefAlignController.atGoal()))
        .andThen(superstructure.setWantedSuperStateCommand(SuperState.L4))
        .andThen(waitSeconds(.5));
  }

  private Command AutoAlignL3Score(PathPlannerPath segment) {
    return superstructure
        .setWantedSuperStateCommand(SuperState.L3)
        .andThen(
            new InstantCommand(
                () -> {
                  autonreefAlignController =
                      new AutonReefAlignController(
                          drive,
                          () ->
                              RobotState.getInstance().getDistanceToNearestReef(drive.getPose())
                                  < 1.0,
                          AllianceFlipUtil.apply(
                              RobotState.getInstance()
                                  .getNearestReefPose(
                                      segment.getIdealTrajectory(null).get().getEndState().pose)));
                }))
        .andThen(
            new InstantCommand(
                    () -> {
                      drive.runVelocity(autonreefAlignController.update().get());
                    },
                    drive)
                .repeatedly()
                .until(
                    () ->
                        autonreefAlignController.atGoal()
                            && Superstructure.getCurrentState() == SuperState.L3))
        .andThen(waitSeconds(.1));
  }

  private Command AutoAlignL2Score(PathPlannerPath segment) {
    return superstructure
        .setWantedSuperStateCommand(SuperState.L2)
        .andThen(
            new InstantCommand(
                () -> {
                  autonreefAlignController =
                      new AutonReefAlignController(
                          drive,
                          () ->
                              RobotState.getInstance().getDistanceToNearestReef(drive.getPose())
                                  < 1.0,
                          AllianceFlipUtil.apply(
                              RobotState.getInstance()
                                  .getNearestReefPose(
                                      segment.getIdealTrajectory(null).get().getEndState().pose)));
                }))
        .andThen(
            new InstantCommand(
                    () -> {
                      drive.runVelocity(autonreefAlignController.update().get());
                    },
                    drive)
                .repeatedly()
                .until(
                    () ->
                        autonreefAlignController.atGoal()
                            && Superstructure.getCurrentState() == SuperState.L2))
        .andThen(waitSeconds(.1));
  }

  // Auto init helpers
  private Command resetPose(final PathPlannerPath segment) {
    return runOnce(
        () -> {

          // var correctedTraj =
          //     segment.generateTrajectory(new ChassisSpeeds(), new Rotation2d(), null);
          // Pose2d pose = correctedTraj.getInitialPose();
          Pose2d pose = AllianceFlipUtil.apply(segment.getStartingHolonomicPose().get());
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
        .setWantedSuperStateCommand(SuperState.L2PREPARE)
        .alongWith(follow(loadSegment(start, end)));
  }

  private Command StowFollow(PathPlannerPath path) {
    return superstructure.setWantedSuperStateCommand(SuperState.L3PREPARE).alongWith(follow(path));
  }

  private Command L3PrepareFollow(final Location start, final Location end) {
    return L3PrepareFollow(loadSegment(start, end));
  }

  private Command L3PrepareFollow(PathPlannerPath path) {
    return superstructure.setWantedSuperStateCommand(SuperState.INTAKE).alongWith(follow(path));
  }

  private Command IntakeFollow(PathPlannerPath path) {
    return superstructure.setWantedSuperStateCommand(SuperState.INTAKE).alongWith(follow(path));
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
      path = PathPlannerPath.fromChoreoTrajectory(name);
    } catch (Exception e) {
      e.printStackTrace();
      path = null;
    }

    path.preventFlipping = false;

    // return new AutoSegment(start, end, name, path);
    return path;
  }
}
