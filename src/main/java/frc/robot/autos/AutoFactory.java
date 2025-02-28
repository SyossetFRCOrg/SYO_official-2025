// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import lombok.RequiredArgsConstructor;
import frc.robot.AutoSelector.AutoQuestionResponse;
import frc.robot.commands.AutonReefAlignController;
import frc.robot.commands.ReefAlignController;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Container;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;

@RequiredArgsConstructor
public class AutoFactory {
  private final Drive drive;
  private final Superstructure superstructure;
  private final Supplier<List<AutoQuestionResponse>> responses;

  private AutonReefAlignController autonreefAlignController;
  private boolean trajectoriesLoaded = false;





  private static final HashMap<AutoQuestionResponse, List<Location>> responseToLocationMap = responseToLocationMap();

  private static final HashMap<AutoQuestionResponse, List<Location>> responseToLocationMap() {
    var map = new HashMap<AutoQuestionResponse, List<Location>>();
    // to be tuned
    map.put(AutoQuestionResponse.FAR_LEFT, new ArrayList<>() {{ add(Location.FARLEFTSTART); }});

    map.put(AutoQuestionResponse.MID_LEFT, new ArrayList<>() {{ add(Location.MIDLEFTSTART); }});

    map.put(AutoQuestionResponse.MID_RIGHT, new ArrayList<>() {{ add(Location.MIDRIGHTSTART); }});

    map.put(AutoQuestionResponse.FAR_RIGHT, new ArrayList<>() {{ add(Location.FARRIGHTSTART); }});

    map.put(AutoQuestionResponse.LEFT_BACK_CORALSTATION, new ArrayList<>() {{ add(Location.LEFTBACKCORALSTATION); }});

    map.put(AutoQuestionResponse.LEFT_FORWARD_CORALSTATION, new ArrayList<>() {{ add(Location.LEFTFORWARDCORALSTATION); }});

    map.put(AutoQuestionResponse.RIGHT_BACK_CORALSTATION, new ArrayList<>() {{ add(Location.RIGHTBACKCORALSTATION); }});

    map.put(AutoQuestionResponse.RIGHT_FORWARD_CORALSTATION, new ArrayList<>() {{ add(Location.RIGHTFORWARDCORALSTATION); }});


    map.put(AutoQuestionResponse.A, new ArrayList<>() {{ add(Location.PREA); add(Location.A);}});

    map.put(AutoQuestionResponse.B, new ArrayList<>() {{ add(Location.PREB); add(Location.B); }});

    map.put(AutoQuestionResponse.C, new ArrayList<>() {{ add(Location.PREC); add(Location.C); }});

    map.put(AutoQuestionResponse.D, new ArrayList<>() {{ add(Location.PRED); add(Location.D); }});

    map.put(AutoQuestionResponse.E, new ArrayList<>() {{ add(Location.PREE); add(Location.E); }});

    map.put(AutoQuestionResponse.F, new ArrayList<>() {{ add(Location.PREF); add(Location.F); }});

    map.put(AutoQuestionResponse.G, new ArrayList<>() {{ add(Location.PREG); add(Location.G); }});

    map.put(AutoQuestionResponse.H, new ArrayList<>() {{ add(Location.PREH); add(Location.H); }});

    map.put(AutoQuestionResponse.I, new ArrayList<>() {{ add(Location.PREI); add(Location.I); }});

    map.put(AutoQuestionResponse.J, new ArrayList<>() {{ add(Location.PREJ); add(Location.J); }});

    map.put(AutoQuestionResponse.K, new ArrayList<>() {{ add(Location.PREK); add(Location.K); }});

    map.put(AutoQuestionResponse.L, new ArrayList<>() {{ add(Location.PREL); add(Location.L); }});

    return map;
  }
  /**
   *
   * @param response
   * @return list of Location datatype corresponding to the response. If it's a reef face, list.get(0)
   * is for the "PRE" location of the path and list.get(1) is the actual single letter location.
   */
  private List<Location> getLocationforQuestion(AutoQuestionResponse response){
    return responseToLocationMap.get(response);

  //   List<Location> list = new ArrayList<>();

  //   //holy switch case
  //   switch (response){
  //   case A:
  //   {
  //     list.add(Location.PREA);
  //     list.add(Location.A);
  //     break;
  //   }
  //   case B:
  //   {
  //     list.add(Location.PREB);
  //     list.add(Location.B);
  //     break;
  //   }
  //   case C:
  //   {
  //     list.add(Location.PREC);
  //     list.add(Location.C);
  //     break;
  //   }
  //   case D:
  //   {
  //     list.add(Location.PRED);
  //     list.add(Location.D);
  //     break;
  //   }
  //   case E:
  //   {
  //     list.add(Location.PREE);
  //     list.add(Location.E);
  //     break;
  //   }
  //   case F:
  //   {
  //     list.add(Location.PREF);
  //     list.add(Location.F);
  //     break;
  //   }
  //   case G:
  //   {
  //     list.add(Location.PREG);
  //     list.add(Location.G);
  //     break;
  //   }
  //   case H:
  //   {
  //     list.add(Location.PREH);
  //     list.add(Location.H);
  //     break;
  //   }
  //   case I:
  //   {
  //     list.add(Location.PREI);
  //     list.add(Location.I);
  //     break;
  //   }
  //   case J:
  //   {
  //     list.add(Location.PREJ);
  //     list.add(Location.J);
  //     break;
  //   }
  //   case K:
  //   {
  //     list.add(Location.PREK);
  //     list.add(Location.K);
  //     break;
  //   }
  //   case L:
  //   {
  //     list.add(Location.PREL);
  //     list.add(Location.L);
  //     break;
  //   }
    
  //   case LEFT_BACK_CORALSTATION:
  //   {
  //     list.add(Location.LEFTBACKCORALSTATION);
  //     break;
  //   }
  //   case LEFT_FORWARD_CORALSTATION:
  //   {
  //     list.add(Location.LEFTFORWARDCORALSTATION);
  //     break;
  //   }
  //   case RIGHT_BACK_CORALSTATION:
  //   {
  //     list.add(Location.RIGHTBACKCORALSTATION);
  //     break;
  //   }
  //   case RIGHT_FORWARD_CORALSTATION:
  //   {
  //     list.add(Location.RIGHTFORWARDCORALSTATION);
  //     break;
  //   }
  //   default:
  //   {
  //     list.add(Location.NONE);
  //     break;
  //   }
  // }
  //   return list;    
  } 



  /* Autonomous program factories
   *
   * Factory methods should be added here for each autonomous program.
   * The factory methods must:
   *   1. Be package-private (i.e. no access modifier)
   *   2. Accept no parameters
   *   3. Return a link Command
   */
  private Command createIdleCommand() {
    // return superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.STOPPED);
    return Commands.none();
  }

  public Command FourAuto() {
    PathPlannerPath firstSegment = loadSegment(getLocationforQuestion(responses.get().get(0)).get(0), getLocationforQuestion(responses.get().get(1)).get(0));
    preloadTrajectoryClass(firstSegment);

    SequentialCommandGroup c = new SequentialCommandGroup();
    c.addCommands(resetPose(firstSegment));
   
    for (int i=0; i<4; i++){
      PathPlannerPath firstpath = loadSegment(getLocationforQuestion(responses.get().get(3 * i)).get(0), getLocationforQuestion(responses.get().get(3 * i + 1)).get(0));
      preloadTrajectoryClass(firstpath);

      c.addCommands(StowFollow(firstpath));

      final AutoQuestionResponse resp = responses.get().get(4*i+2);
      c.addCommands(
        Commands.select(
          Map.of(
          AutoQuestionResponse.L4,
          AutoAlignL4Score(firstpath),
          AutoQuestionResponse.L3,
          AutoAlignL3Score(firstpath),
          AutoQuestionResponse.L2,
          AutoAlignL2Score(firstpath)),
          () -> resp));
      
      c.addCommands(IntakeFollow(getLocationforQuestion(responses.get().get(3 * i + 1)).get(1), getLocationforQuestion(responses.get().get(3 * i + 3)).get(0)));
    }
    return c;
  }

  public Command ThreeAuto() {
    PathPlannerPath firstSegment = loadSegment(getLocationforQuestion(responses.get().get(0)).get(0), getLocationforQuestion(responses.get().get(1)).get(0));
    preloadTrajectoryClass(firstSegment);

    SequentialCommandGroup c = new SequentialCommandGroup();
    c.addCommands(resetPose(firstSegment));
   
    for (int i=0; i<3; i++){
      PathPlannerPath firstpath = loadSegment(getLocationforQuestion(responses.get().get(3 * i)).get(0), getLocationforQuestion(responses.get().get(3 * i + 1)).get(0));
      c.addCommands(StowFollow(firstpath));

      final AutoQuestionResponse resp = responses.get().get(4*i+2);
      c.addCommands(
        Commands.select(
          Map.of(
          AutoQuestionResponse.L4,
          AutoAlignL4Score(firstpath),
          AutoQuestionResponse.L3,
          AutoAlignL3Score(firstpath),
          AutoQuestionResponse.L2,
          AutoAlignL2Score(firstpath)),
          () -> resp));
      
      c.addCommands(IntakeFollow(getLocationforQuestion(responses.get().get(3 * i + 1)).get(1), getLocationforQuestion(responses.get().get(3 * i + 3)).get(0)));
    }
    return c;
  }

  public Command TwoAuto() {
    PathPlannerPath firstSegment = loadSegment(getLocationforQuestion(responses.get().get(0)).get(0), getLocationforQuestion(responses.get().get(1)).get(0));
    preloadTrajectoryClass(firstSegment);

    SequentialCommandGroup c = new SequentialCommandGroup();
    c.addCommands(resetPose(firstSegment));
   
    for (int i=0; i<4; i++){
      PathPlannerPath firstpath = loadSegment(getLocationforQuestion(responses.get().get(3 * i)).get(0), getLocationforQuestion(responses.get().get(3 * i + 1)).get(0));
      c.addCommands(StowFollow(firstpath));

      final AutoQuestionResponse resp = responses.get().get(4*i+2);
      c.addCommands(
        Commands.select(
          Map.of(
          AutoQuestionResponse.L4,
          AutoAlignL4Score(firstpath),
          AutoQuestionResponse.L3,
          AutoAlignL3Score(firstpath),
          AutoQuestionResponse.L2,
          AutoAlignL2Score(firstpath)),
          () -> resp));
      
      c.addCommands(IntakeFollow(getLocationforQuestion(responses.get().get(3 * i + 1)).get(1), getLocationforQuestion(responses.get().get(3 * i + 3)).get(0)));
    }
    return c;
  }
  
  private Command AutoAlignL4Score(PathPlannerPath segment) {
    return
    superstructure
        .setWantedSuperStateCommand(SuperState.L4)
        .andThen(
    new InstantCommand(
            () -> {
              autonreefAlignController =
                  new AutonReefAlignController(
                      drive,
                      () ->
                          RobotState.getInstance().getDistanceToNearestReef(drive.getPose()) < 1.0,
                      RobotState.getInstance().getNearestReefPose(segment.getIdealTrajectory(null).get().getEndState().pose));
            })
        )
        .andThen(
            new InstantCommand(
                    () -> {
                      drive.runVelocity(autonreefAlignController.update().get());
                    },
                    drive)
                .repeatedly()
                .until(
                    () -> autonreefAlignController.atGoal()
                    && Superstructure.getCurrentState() == SuperState.L4
                    ))
        .andThen(waitSeconds(.4));
  }

  private Command AutoAlignL3Score(PathPlannerPath segment) {
    return
    superstructure
        .setWantedSuperStateCommand(SuperState.L3)
        .andThen(
    new InstantCommand(
            () -> {
              autonreefAlignController =
              new AutonReefAlignController(
                  drive,
                  () ->
                      RobotState.getInstance().getDistanceToNearestReef(drive.getPose()) < 1.0,
                  RobotState.getInstance().getNearestReefPose(segment.getIdealTrajectory(null).get().getEndState().pose));
        
            })
        )
        .andThen(
            new InstantCommand(
                    () -> {
                      drive.runVelocity(autonreefAlignController.update().get());
                    },
                    drive)
                .repeatedly()
                .until(
                    () -> autonreefAlignController.atGoal()
                    && Superstructure.getCurrentState() == SuperState.L3
                    ))
        .andThen(waitSeconds(.4));
  }

  private Command AutoAlignL2Score(PathPlannerPath segment) {
    return
    superstructure
        .setWantedSuperStateCommand(SuperState.L2)
        .andThen(
    new InstantCommand(
            () -> {
              autonreefAlignController =
                  new AutonReefAlignController(
                      drive,
                      () ->
                          RobotState.getInstance().getDistanceToNearestReef(drive.getPose()) < 1.0,
                      RobotState.getInstance().getNearestReefPose(segment.getIdealTrajectory(null).get().getEndState().pose));
            
            })
        )
        .andThen(
            new InstantCommand(
                    () -> {
                      drive.runVelocity(autonreefAlignController.update().get());
                    },
                    drive)
                .repeatedly()
                .until(
                    () -> autonreefAlignController.atGoal()
                    && Superstructure.getCurrentState() == SuperState.L2
                    ))
        .andThen(waitSeconds(.4));
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
    return
    superstructure
        .setWantedSuperStateCommand(SuperState.STOW)
        .alongWith(
    follow(loadSegment(start, end))
    )
    ;
  }
  private Command StowFollow(PathPlannerPath path) {
    return
    superstructure
        .setWantedSuperStateCommand(SuperState.STOW)
        .alongWith(
    follow(path)
    )
    ;
  }

  private Command IntakeFollow(final Location start, final Location end) {
    return
    (superstructure
        .setWantedSuperStateCommand(SuperState.INTAKE)
        .alongWith(
    follow(loadSegment(start, end))
    ));
  }

  // Path following
  private Command follow(final Location start, final Location end) {
    return follow(loadSegment(start, end));
  }

  // Path following
  private Command follow(PathPlannerPath path) {
    return com.pathplanner.lib.auto.AutoBuilder.followPath(path);
  }

  private void preloadTrajectoryClass(PathPlannerPath firstSegment) {
    // This is done because Java loads classes lazily. Calling this here loads the trajectory class
    // which is used to follow paths and saves user code ms loop time at the start of auto.
    if (!trajectoriesLoaded) {
      trajectoriesLoaded = true;
      var trajectory =
          new PathPlannerTrajectory(
              firstSegment, drive.getChassisSpeeds(), drive.getPose().getRotation(), Drive.PP_CONFIG);
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
