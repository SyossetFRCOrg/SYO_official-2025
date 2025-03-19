package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
// import frc.robot.subsystems.climber.ClimberSubsystem;
// import frc.robot.config.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

// import frc.robot.subsystems.shooter.ShooterSubsystem;
// import frc.robot.subsystems.swerve.SwerveSubsystem;
// import frc.robot.subsystems.turret.TurretSubsystem;

public class Superstructure extends SubsystemBase {
  private Drive drive;
  private Elevator elevator;
  private RobotContainer container;
  private Wrist wrist;

  // private ClimberSubsystem climber;

  // private final Timer climberHeadingLockTimer = new Timer();

  // private boolean subwooferShotMode = false;
  // private boolean feedShotMode = false;

  public static enum SuperState {
    // MANUAL,
    INTAKEPREPARE,
    INTAKE,
    INTAKELOW,
    INTAKELOWPREPARE,
    L1,
    L2,
    L3,
    L4,
    L1PREPARE,
    L2PREPARE,
    L3PREPARE,
    L4PREPARE,
    STOPPED,
    STOW,
    L2L3ALGAE,
    L3L4ALGAE
  }

  private static @Getter @Setter SuperState desiredState = SuperState.STOW;
  private static @Getter @Setter SuperState currentState = SuperState.STOW;
  private static SuperState previousState = SuperState.STOW;

  // RobotState.AimingParameters aimingParameters =
  //         new RobotState.AimingParameters(new Rotation2d(), new Rotation2d(), new
  // Translation2d(), 0.0);

  // private static final double CLIMBER_MOTOR_ROTATIONS_CLIMB = 105.0;
  // private static final double CLIMBER_MOTOR_ROTATIONS_CLIMB_SECONDARY = 45.0;
  // private Rotation2d manualTurretSetpoint = new Rotation2d();
  // private Rotation2d manualPitchSetpoint = new Rotation2d();

  public Superstructure(Drive drive, Elevator elevator, Wrist wrist, RobotContainer container) {
    this.drive = drive;
    this.elevator = elevator;
    this.container = container;
    this.wrist = wrist;
  }

  @Override
  public void periodic() {
    // complex logging stuff for later on...? If we want to

    // double percentageOfThreeMetersPerSecond = Math.hypot(
    //                 RobotState.getInstance().getChassisSpeeds().vxMetersPerSecond,
    //                 RobotState.getInstance().getChassisSpeeds().vyMetersPerSecond)
    //         / 3.0;
    // aimingParameters = RobotState.getInstance()
    //         .getAimingParameters(0.6 * percentageOfThreeMetersPerSecond, 0.25 *
    // percentageOfThreeMetersPerSecond);

    currentState = handleStateTransitions();

    // janky way of logging robotstate values. Robot state @AutoLog doesn't work???
    Logger.recordOutput("RobotState/aboveL1", RobotState.getInstance().isAboveL1());

    Logger.recordOutput(
        "RobotState/elevatorPosition", RobotState.getInstance().getElevatorPosition());

    Logger.recordOutput("RobotState/addingVision", RobotState.getInstance().isAddingVision());

    Logger.recordOutput("RobotState/wristCanMove", RobotState.getInstance().isWristCanMove());

    Logger.recordOutput(
        "RobotState/reefAutoAligning", RobotState.getInstance().isReefAutoAligning());

    Logger.recordOutput("RobotState/reefAutoAiming", RobotState.getInstance().isReefAutoAiming());

    Logger.recordOutput("RobotState/isLSwitching", RobotState.getInstance().isLimitSwitching());
    Logger.recordOutput(
        "RobotState/intakeAutoAiming", RobotState.getInstance().isIntakeAutoAiming());

    if (RobotState.getInstance().getTuningTempPose() != null) {
      Logger.recordOutput(
          "RobotState/tuningTempPose",
          new double[] {
            RobotState.getInstance().getTuningTempPose().getX(),
            RobotState.getInstance().getTuningTempPose().getY(),
            RobotState.getInstance().getTuningTempPose().getRotation().getDegrees()
          });
    } else {
      Logger.recordOutput("RobotState/tuningTempPose", new double[] {0, 0, 0});
    }

    Logger.recordOutput(
        "Drive/EstimatedPose",
        new double[] {
          drive.getPose().getX(), drive.getPose().getY(), drive.getPose().getRotation().getDegrees()
        });

    Logger.recordOutput("Superstructure/CurrentSuperState", currentState.toString());
    Logger.recordOutput("Superstructure/DesiredSuperState", desiredState.toString());

    if (currentState == SuperState.STOPPED) handleStopped();

    // Logger.recordOutput("TeleopShotReady/PivotAtSetpoint", pivot.pivotAtSetpoint());
    // Logger.recordOutput("TeleopShotReady/PivotGreaterThan10", pivot.getCurrentPosition() > 10.0);
    // Logger.recordOutput("TeleopShotReady/ShooterAtSpeakerSetpoint", shooter.atSpeakerSetpoint());
    // Logger.recordOutput(
    //         "TeleopShotReady/AccelerationVectorUnder12",
    //         RobotState.getInstance().getLastAccelerationVector() < 0.12);
    // Logger.recordOutput(
    //         "TeleopShotReady/HasTarget", RobotState.getInstance().hasTarget());
    // Logger.recordOutput(
    //         "TeleopShotReady/CameraWithin8Meters",
    // RobotState.getInstance().getVisionHorizontalDistance() <= 8.0);
    // Logger.recordOutput(
    //         "TeleopShotReady/PredictedPoseWithin8Meters",
    //         aimingParameters.effectiveDistance().getX() <= 8.0);

    // Logger.recordOutput("DesiredSuperstate", desiredState);
    // if (currentState != previousState) {
    //     Logger.recordOutput("CurrentSuperstate", currentState);
    // }

    // Logger.recordOutput(
    //         "AimingParameters/AdjustedTurretAngleDegrees",
    //         aimingParameters.turretAimingAngle().getDegrees());
    // Logger.recordOutput("AimingParameters/EffectiveDistance",
    // aimingParameters.effectiveDistance());

    // Logger.recordOutput("FeedShotDistance", RobotState.getInstance().getDistanceToFeedTarget());
  }

  /**
   * Sets currentState to the appropiate transition state based on desiredState
   *
   * @return The current super state
   */
  private SuperState handleStateTransitions() {
    previousState = currentState;
    var ready = ready(desiredState);
    currentState =
        switch (desiredState) {
          case L1 -> ready ? SuperState.L1 : SuperState.L1PREPARE;
          case L2 -> ready ? SuperState.L2 : SuperState.L2PREPARE;
          case L3 -> ready ? SuperState.L3 : SuperState.L3PREPARE;
          case L4 -> ready ? SuperState.L4 : SuperState.L4PREPARE;
          case INTAKE -> ready ? SuperState.INTAKE : SuperState.INTAKEPREPARE;
          case INTAKELOW -> ready ? SuperState.INTAKELOW : SuperState.INTAKELOWPREPARE;
          default -> desiredState;
        };

    return currentState;
  }

  private void handleStopped() {
    drive.stop();
    elevator.stop();
  }

  /** Transition check */
  private boolean ready(SuperState state) {
    return switch (state) {
        // also has to be at alignment goal to score
      case L2, L3, L4 -> elevator.atSetPoint(state) && wrist.atSetPoint(state);
        // && container.getReefAlignController().atGoal();
        // L1 is prospectively manual driving alignment
      case L1 -> elevator.atSetPoint(state) && wrist.atSetPoint(state);
      case INTAKE -> elevator.atSetPoint(state);
      case INTAKELOW -> elevator.atSetPoint(state);
      case STOW, INTAKEPREPARE, L2L3ALGAE, L3L4ALGAE -> true;
      default -> false;
    };
  }

  public BooleanSupplier doesCommandMatch(SuperState currentState) {
    return () -> Superstructure.currentState == currentState;
  }

  /** State pushers */
  public void setWantedSuperState(SuperState desiredState) {
    Superstructure.desiredState = desiredState;
  }

  public Command setWantedSuperStateCommand(SuperState desiredState) {
    return new InstantCommand(
        () -> {
          setWantedSuperState(desiredState);
        });
  }
}

// package frc.robot.subsystems;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.RobotState;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.SynchronousInterrupt.WaitResult;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.climber.ClimberSubsystem;
// // import frc.robot.config.FieldConstants;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.flywheel.Flywheel;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.vision.Vision;

// import static edu.wpi.first.wpilibj2.command.Commands.none;
// import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

// // import frc.robot.subsystems.shooter.ShooterSubsystem;
// // import frc.robot.subsystems.swerve.SwerveSubsystem;
// // import frc.robot.subsystems.turret.TurretSubsystem;
// import org.littletonrobotics.junction.Logger;

// import com.fasterxml.jackson.annotation.JsonTypeInfo.None;

// public class Superstructure extends SubsystemBase {

//     private Drive drive;
//     private Flywheel flywheel;
//     private Intake intake;
//     private ClimberSubsystem climber;
//     private RobotContainer container;

//     // private final Timer climberHeadingLockTimer = new Timer();

//     private boolean subwooferShotMode = false;
//     private boolean feedShotMode = false;

//     public static enum SuperState {
//         HOLD_FIX_PIECE,
//         REGULAR_STATE,
//         PREPARING_SUBWOOFER_SHOT,
//         SUBWOOFER_SHOT,
//         PREPARING_LIMELIGHT_SHOT,
//         LIMELIGHT_SHOT,
//         PREPARING_PASS,
//         PASS,

//         INTAKE_DOWN,
//         INTAKE_UP,
//         CLIMBER_UP,
//         CLIMBER_DOWN,
//         STOPPED,
//     }

//     public static enum SuperState {
//         HOLD_FIX_PIECE,
//         REGULAR_STATE,
//         PREPARING_SUBWOOFER_SHOT,
//         SUBWOOFER_SHOT,
//         PREPARING_LIMELIGHT_SHOT,
//         LIMELIGHT_SHOT,
//         PREPARING_PASS,
//         PASS,

//         INTAKE_DOWN,
//         INTAKE_UP,
//         CLIMBER_UP,
//         CLIMBER_DOWN,
//         STOPPED,
//     }

//     private SuperState desiredState = SuperState.STOPPED;
//     public static SuperState currentState = SuperState.STOPPED;
//     private SuperState previousState;
//     // RobotState.AimingParameters aimingParameters =
//     //         new RobotState.AimingParameters(new Rotation2d(), new Rotation2d(), new
// Translation2d(), 0.0);

//     // private static final double CLIMBER_MOTOR_ROTATIONS_CLIMB = 105.0;
//     // private static final double CLIMBER_MOTOR_ROTATIONS_CLIMB_SECONDARY = 45.0;
//     // private Rotation2d manualTurretSetpoint = new Rotation2d();
//     // private Rotation2d manualPitchSetpoint = new Rotation2d();

//     public Superstructure(
//         Drive drive,
//         Flywheel flywheel,
//         Intake intake,
//         ClimberSubsystem climber,

//         RobotContainer container

//             ) {
//         this.drive = drive;
//         this.flywheel = flywheel;
//         this.intake = intake;
//         this.climber = climber;
//         this.container = container;

//     }

//     @Override
//     public void periodic() {
//         // double percentageOfThreeMetersPerSecond = Math.hypot(
//         //                 RobotState.getInstance().getChassisSpeeds().vxMetersPerSecond,
//         //                 RobotState.getInstance().getChassisSpeeds().vyMetersPerSecond)
//         //         / 3.0;
//         // aimingParameters = RobotState.getInstance()
//         //         .getAimingParameters(0.6 * percentageOfThreeMetersPerSecond, 0.25 *
// percentageOfThreeMetersPerSecond);
//         currentState = handleStateTransitions();
//         applyStates();

//         // Logger.recordOutput("TeleopShotReady/PivotAtSetpoint", pivot.pivotAtSetpoint());
//         // Logger.recordOutput("TeleopShotReady/PivotGreaterThan10", pivot.getCurrentPosition() >
// 10.0);
//         // Logger.recordOutput("TeleopShotReady/ShooterAtSpeakerSetpoint",
// shooter.atSpeakerSetpoint());
//         // Logger.recordOutput(
//         //         "TeleopShotReady/AccelerationVectorUnder12",
//         //         RobotState.getInstance().getLastAccelerationVector() < 0.12);
//         // Logger.recordOutput(
//         //         "TeleopShotReady/HasTarget", RobotState.getInstance().hasTarget());
//         // Logger.recordOutput(
//         //         "TeleopShotReady/CameraWithin8Meters",
// RobotState.getInstance().getVisionHorizontalDistance() <= 8.0);
//         // Logger.recordOutput(
//         //         "TeleopShotReady/PredictedPoseWithin8Meters",
//         //         aimingParameters.effectiveDistance().getX() <= 8.0);

//         // Logger.recordOutput("DesiredSuperstate", desiredState);
//         // if (currentState != previousState) {
//         //     Logger.recordOutput("CurrentSuperstate", currentState);
//         // }

//         // Logger.recordOutput(
//         //         "AimingParameters/AdjustedTurretAngleDegrees",
//         //         aimingParameters.turretAimingAngle().getDegrees());
//         // Logger.recordOutput("AimingParameters/EffectiveDistance",
// aimingParameters.effectiveDistance());

//         // Logger.recordOutput("FeedShotDistance",
// RobotState.getInstance().getDistanceToFeedTarget());
//     }

//     private SuperState handleStateTransitions() {
//         previousState = currentState;
//         switch (desiredState) {
//             case HOLD_FIX_PIECE:

//                 currentState = SuperState.HOLD_FIX_PIECE;
//                 break;

//             case REGULAR_STATE:
//                 currentState = SuperState.REGULAR_STATE;
//                 break;
//             case PREPARING_SUBWOOFER_SHOT:
//                 currentState = SuperState.PREPARING_SUBWOOFER_SHOT;
//                 break;
//             // case READY_FOR_SUBWOOFER_SHOT:
//             //     currentState = SuperState.READY_FOR_SUBWOOFER_SHOT;
//             //     break;
//             case SUBWOOFER_SHOT:
//                 currentState = areSystemsReadyForSubwooferShot()
//                         ? SuperState.SUBWOOFER_SHOT
//                         : SuperState.PREPARING_SUBWOOFER_SHOT;

//                 break;
//             case PREPARING_LIMELIGHT_SHOT:
//                 currentState = SuperState.PREPARING_LIMELIGHT_SHOT;
//                 break;
//             // case READY_FOR_LIMELIGHT_SHOT:
//             //     currentState = SuperState.READY_FOR_LIMELIGHT_SHOT;
//             //     break;
//             case LIMELIGHT_SHOT:
//                 currentState = areSystemsReadyForLimelightShot()
//                 ? SuperState.LIMELIGHT_SHOT
//                 : SuperState.PREPARING_LIMELIGHT_SHOT;
//                 break;
//             case PREPARING_PASS:
//                 currentState = SuperState.PREPARING_PASS;
//                 break;
//             case PASS:

//                 currentState = areSystemsReadyForPassShot() ? SuperState.PASS :
// SuperState.PREPARING_PASS;
//                 break;

//             // case READY_FOR_INTAKE:
//             //     currentState = SuperState.READY_FOR_INTAKE;
//             //     break;
//             case INTAKE_DOWN:
//                 currentState = SuperState.INTAKE_DOWN;
//                 break;
//             case INTAKE_UP:
//                 currentState = SuperState.INTAKE_UP;
//                 break;
//             case CLIMBER_UP:
//                 currentState = SuperState.CLIMBER_UP;
//                 break;
//             case CLIMBER_DOWN:
//                 currentState = SuperState.CLIMBER_DOWN;
//                 break;

//             case STOPPED:
//             default:
//                 currentState = SuperState.STOPPED;
//                 break;

//         }
//         return currentState;
//     }

//     private void applyStates() {
//         switch (currentState) {
//             case REGULAR_STATE:
//                 drive.disableRotationLock();
//                 makeSureIntakeUp(true);
//             case HOLD_FIX_PIECE:
//                 holdFixPiece();
//                 break;
//             case PREPARING_SUBWOOFER_SHOT:
//                 prepareForSubwooferShot();
//                 break;
//             case SUBWOOFER_SHOT:
//                 subwooferShot();
//                 break;
//             case PREPARING_LIMELIGHT_SHOT:
//                 prepareForLimelightShot();
//                 break;
//             case LIMELIGHT_SHOT:
//                 limelightShot();
//                 break;
//             case PREPARING_PASS:
//                 prepareForPass();
//                 break;
//             case PASS:
//                 pass();
//                 break;
//             case INTAKE_DOWN:
//                 intakeDown();
//                 break;
//             case INTAKE_UP:
//                 intakeUp();
//                 break;
//             case CLIMBER_DOWN:
//             case CLIMBER_UP:
//                 climb();
//                 break;
//             case STOPPED:
//             default:
//                 handleStopped();
//                 break;
//         }
//     }

//     /**
//      * checks
//      */
//     private boolean areSystemsReadyForSubwooferShot(){
//         boolean isReady = flywheel.atSetpoint(flywheel.getSubwooferAngle(),
//         flywheel.getsubwooferRPM(),
//         desiredState)
//                 && intake.atShootPoint();

//         return isReady;
//     }

//     /** Checks */
//     private boolean areSystemsReadyForLimelightShot() {
//         boolean isReady = flywheel.atSetpoint(drive.calculateShootAngle(),
//         flywheel.getLimelightAndPassRPM(),
//         desiredState)
//                 && intake.atShootPoint()
//                 && drive.atShootSetPoint()
//                 && drive.stopped();

//         return isReady;
//     }

//     /** checks

//     */
//     private boolean areSystemsReadyForPassShot() {
//         boolean isReady = flywheel.atSetpoint(drive.calculatePassAngle(),
//         flywheel.getMaxOuttakeRate() * .75,
//         desiredState)
//                 && intake.atShootPoint()
//                 && drive.atPassSetPoint()
//                 && drive.stopped();
//         return isReady;
//     }

//     /** moves intake to shootpoint if it isn't yet*/
//     private void makeSureIntakeUp(boolean revUp) {
//         if (!intake.atShootPoint())
//         {
//             Commands.run(() -> intake.rotate(() -> 0.0));
//         }
//         if (revUp)
//         {
//             Commands.run(() -> flywheel.runVelocity(flywheel.getIdleRPM()));

//         }
//         Commands.run(() -> flywheel.aim(flywheel.getTravelAngle()));

//     }

//     /**
//      * moves the note up and down and up and down rapidly to fix it while it moves the intake
// back up.
//      * therefore, the note should be nicely intaked by the time it's tdone
//      */
//     private void intakeUp(){

//         Commands.runOnce(() -> Commands.deadline(

//         Commands.runOnce(() -> makeSureIntakeUp(true)).until(() -> intake.atShootPoint()),

//         new SequentialCommandGroup(
//             Commands.deadline(waitSeconds(.25),  Commands.run(() -> intake.intake( () ->
// 700.0))),
//             Commands.deadline(waitSeconds(.3),  Commands.run(() -> intake.intake( () -> -700.0)))
//         ).repeatedly()

//         ).finallyDo(() -> Commands.deadline(waitSeconds(.15),  Commands.run(() -> intake.intake(
// () -> -700.0)))));

//         intake.intake(() -> 0);
//         desiredState = SuperState.REGULAR_STATE;
//     }

//     /**
//      * moves the note up and down and up and down rapidly to fix it while it moves the intake
// back up.
//      * therefore, the note should be nicely intaked by the time it's tdone
//      */
//     private void holdFixPiece(){

//         Commands.runOnce(() -> Commands.deadline(

//         waitSeconds(1),

//         new SequentialCommandGroup(
//             Commands.deadline(waitSeconds(.25),  Commands.run(() -> intake.intake( () ->
// 700.0))),
//             Commands.deadline(waitSeconds(.3),  Commands.run(() -> intake.intake( () -> -700.0)))
//         ).repeatedly()

//         ).finallyDo(() -> Commands.deadline(waitSeconds(.15),  Commands.run(() -> intake.intake(
// () -> -700.0)))));

//         intake.intake(() -> 0);
//         desiredState = SuperState.REGULAR_STATE;
//     }

//     /**
//      * moves the intake up and revs the shooter up while bringing it to the angle for subwoofer
// shoot
//      */
//     private void prepareForSubwooferShot(){
//         // the preparing should continue regardless until it's just not called anymore
//         // if (!areSystemsReadyForSubwooferShot())
//         // {
//         Commands.run(() ->  new ParallelCommandGroup(
//         Commands.run(() -> flywheel.aim(flywheel.getSubwooferAngle())),
//         Commands.run(() -> flywheel.runVelocity(flywheel.getsubwooferRPM())),
//         Commands.run(() -> makeSureIntakeUp(false))))
//         ;
//         // }
//     }

//     /**
//      * moves the intake up and revs the shooter up while aiming the shooter and drivetrain for
// limelight shoot
//      */
//     private void prepareForLimelightShot() {
//         // if (!areSystemsReadyForLimelightShot())
//         // {
//             Commands.run( () -> new ParallelCommandGroup(
//             Commands.run(() -> flywheel.aim(drive.calculateShootAngle())),
//             Commands.run(() -> flywheel.runVelocity(flywheel.getLimelightAndPassRPM())),
//             Commands.run(() -> makeSureIntakeUp(false)),
//             Commands.run(() -> drive.setRotationLock())));
//         // }
//     }

//     /**
//      * Does the Subwoofer Shot. sets the robot state to REGULAR_STATE afterwards
//      */
//     private void subwooferShot()
//     {
//         Commands.run( () ->

//         Commands.deadline(waitSeconds(.75),

//         Commands.run(() -> intake.intake( () -> 700.0)),
//         // I don't think the angle should be adjusting during the shot
//         // Commands.run(() -> flywheel.aim(flywheel.getSubwooferAngle())),
//         Commands.run(() -> flywheel.runVelocity(flywheel.getsubwooferRPM())),
//         Commands.run(() -> makeSureIntakeUp(false))
//         )
//         );
//         desiredState = SuperState.REGULAR_STATE;
//         currentState = SuperState.REGULAR_STATE;

//     }

//     /**
//      * Does the limelight Shot. sets the robot state to REGULAR_STATE afterwards
//      */
//     private void limelightShot()
//     {
//         Commands.run( () ->
//         Commands.deadline(waitSeconds(.75),
//         Commands.run(() -> intake.intake( () -> 700.0)),
//         // I don't think the angle should be adjusting during the shot
//         // Commands.run(() -> flywheel.aim(flywheel.getSubwooferAngle())),
//         Commands.run(() -> flywheel.runVelocity(flywheel.getLimelightAndPassRPM())),
//         Commands.run(() -> makeSureIntakeUp(false)),
//         Commands.runOnce(() -> drive.setRotationLock()),
//         Commands.run(() -> drive.stop())
//         )
//         ).andThen(() -> drive.disableRotationLock());

//         desiredState = SuperState.REGULAR_STATE;
//         currentState = SuperState.REGULAR_STATE;
//     }

//      /**
//      * moves the intake up and revs the shooter up while aiming the shooter and drivetrain for
// pass
//      */
//     private void prepareForPass() {
//         // if (!areSystemsReadyForLimelightShot())
//         // {

//             Commands.run (() -> new ParallelCommandGroup(
//             Commands.run(() -> flywheel.aim(flywheel.getPassShotAngle())),
//             Commands.run(() -> flywheel.runVelocity(flywheel.getLimelightAndPassRPM())),
//             Commands.run(() -> makeSureIntakeUp(false)),
//             Commands.run(() -> drive.setRotationLock())));
//         // }
//     }

//     /**
//      * Does the limelight Shot. sets the robot state to REGULAR_STATE afterwards
//      */
//     private void pass()
//     {
//         Commands.run( () ->
//         Commands.deadline(waitSeconds(.75),
//         Commands.run(() -> intake.intake( () -> 700.0)),
//         // I don't think the angle should be adjusting during the shot
//         // Commands.run(() -> flywheel.aim(flywheel.getSubwooferAngle())),
//         Commands.run(() -> flywheel.runVelocity(flywheel.getLimelightAndPassRPM())),
//         Commands.run(() -> makeSureIntakeUp(false)),
//         Commands.runOnce(() -> drive.setRotationLock()),
//         Commands.run(() -> drive.stop())
//         )
//         ).andThen(() -> drive.disableRotationLock());

//         desiredState = SuperState.REGULAR_STATE;
//         currentState = SuperState.REGULAR_STATE;
//     }

//     /**
//      * moves the intake down to the intake angle. This angle is approximated to be the one that
// works,
//      * but may be physically stopped by the bumper, which should be at the right height to be
// able to intake properly
//      * without dragging the intake on the ground
//      *
//      */
//     private void intakeDown(){
//         Commands.runOnce(() -> new ParallelCommandGroup(
//         Commands.run(() -> intake.rotate(() -> -3.3)),
//         Commands.run(() -> intake.intake(() -> -700.0))
//         ));

//     }

//     /**
//      * moves the climber up or down based on the
//      * @param currentState that is passed,
//      * {@value} CLIMBER_UP or
//      * {@value} CLIMBER_DOWN
//      */
//     private void climb(){
//         switch (currentState){
//             case CLIMBER_UP:
//                 Commands.run(() -> climber.climb(.7,.7));
//                 break;
//             case CLIMBER_DOWN:
//                 Commands.run(() -> climber.climb(-.7,-.7));
//                 break;
//             default:
//                 Commands.run(() -> climber.climb(0,0));
//         }
//     }

//     private void handleStopped(){
//         Commands.run(() -> new ParallelCommandGroup(
//         Commands.run(() -> drive.stop()),
//         Commands.run(() -> flywheel.stop()),
//         Commands.run(() -> intake.stop()),
//         Commands.run(() -> climber.stop())
//         ));
//     }

//     /** State pushers */
//     public void setWantedSuperState(SuperState desiredState) {
//         this.desiredState = desiredState;
//     }

//     public Command setWantedSuperStateCommand(SuperState desiredState) {
//         return new InstantCommand(() -> setWantedSuperState(desiredState));
//     }

// }
