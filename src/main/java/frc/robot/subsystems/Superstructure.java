package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
// import frc.robot.subsystems.climber.ClimberSubsystem;
// import frc.robot.config.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.elevator;
import frc.robot.subsystems.elevator.elevator.elevatorState;
import java.util.function.BooleanSupplier;

// import frc.robot.subsystems.shooter.ShooterSubsystem;
// import frc.robot.subsystems.swerve.SwerveSubsystem;
// import frc.robot.subsystems.turret.TurretSubsystem;

public class Superstructure extends SubsystemBase {

  private Drive drive;
  private elevator elevator;

  // private ClimberSubsystem climber;
  private RobotContainer container;

  // private final Timer climberHeadingLockTimer = new Timer();

  private boolean subwooferShotMode = false;
  private boolean feedShotMode = false;

  public static enum WantedSuperState {
    // MANUAL,
    INTAKE,
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
  }

  public static enum CurrentSuperState {
    // MANUAL,
    INTAKE,
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
  }

  private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
  public static CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
  private CurrentSuperState previousSuperState;

  // RobotState.AimingParameters aimingParameters =
  //         new RobotState.AimingParameters(new Rotation2d(), new Rotation2d(), new
  // Translation2d(), 0.0);

  // private static final double CLIMBER_MOTOR_ROTATIONS_CLIMB = 105.0;
  // private static final double CLIMBER_MOTOR_ROTATIONS_CLIMB_SECONDARY = 45.0;
  // private Rotation2d manualTurretSetpoint = new Rotation2d();
  // private Rotation2d manualPitchSetpoint = new Rotation2d();

  public Superstructure(Drive drive, elevator Elevator, RobotContainer container) {

    this.drive = drive;
    this.elevator = Elevator;
    this.container = container;
  }

  @Override
  public void periodic() {
    // double percentageOfThreeMetersPerSecond = Math.hypot(
    //                 RobotState.getInstance().getChassisSpeeds().vxMetersPerSecond,
    //                 RobotState.getInstance().getChassisSpeeds().vyMetersPerSecond)
    //         / 3.0;
    // aimingParameters = RobotState.getInstance()
    //         .getAimingParameters(0.6 * percentageOfThreeMetersPerSecond, 0.25 *
    // percentageOfThreeMetersPerSecond);
    currentSuperState = handleStateTransitions();
    applyStates();

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

    // Logger.recordOutput("DesiredSuperstate", wantedSuperState);
    // if (currentSuperState != previousSuperState) {
    //     Logger.recordOutput("CurrentSuperstate", currentSuperState);
    // }

    // Logger.recordOutput(
    //         "AimingParameters/AdjustedTurretAngleDegrees",
    //         aimingParameters.turretAimingAngle().getDegrees());
    // Logger.recordOutput("AimingParameters/EffectiveDistance",
    // aimingParameters.effectiveDistance());

    // Logger.recordOutput("FeedShotDistance", RobotState.getInstance().getDistanceToFeedTarget());
  }

  private CurrentSuperState handleStateTransitions() {
    previousSuperState = currentSuperState;
    switch (wantedSuperState) {
      case STOW:
        currentSuperState = CurrentSuperState.STOW;
        break;

      case INTAKE:
        currentSuperState = CurrentSuperState.INTAKE;
      case L1PREPARE:
        currentSuperState = CurrentSuperState.L1PREPARE;

      case L1:
        currentSuperState =
            subsystemsL1ready() ? CurrentSuperState.L1 : CurrentSuperState.L1PREPARE;

      case L2PREPARE:
        currentSuperState = CurrentSuperState.L2PREPARE;

      case L2:
        currentSuperState =
            subsystemsL2ready() ? CurrentSuperState.L2 : CurrentSuperState.L2PREPARE;

      case L3PREPARE:
        currentSuperState = CurrentSuperState.L3PREPARE;

      case L3:
        currentSuperState =
            subsystemsL3ready() ? CurrentSuperState.L3 : CurrentSuperState.L3PREPARE;

      case L4PREPARE:
        currentSuperState = CurrentSuperState.L4PREPARE;

      case L4:
        currentSuperState =
            subsystemsL4ready() ? CurrentSuperState.L4 : CurrentSuperState.L4PREPARE;

        // case PASS:
        //     currentSuperState = areSystemsReadyForPassShot()
        //     ? CurrentSuperState.PASS
        //     : CurrentSuperState.PREPARING_PASS;
        //     break;

      case STOPPED:
      default:
        currentSuperState = CurrentSuperState.STOPPED;
        break;
    }
    return currentSuperState;
  }

  private void applyStates() {
    switch (currentSuperState) {
      case INTAKE:
        intake();
        break;
      case L1PREPARE:
        L1prepare();
        break;
      case L1:
        L1();
        break;
      case L2PREPARE:
        L2prepare();
        break;
      case L2:
        L2();
        break;
      case L3PREPARE:
        L3prepare();
        break;
      case L3:
        L3();
        break;
      case L4PREPARE:
        L4prepare();
        break;
      case L4:
        L4();
        break;
      case STOW:
        stow();
        break;

        // case CLIMBER_DOWN:
        //     climbDown();
        //     break;
        // case CLIMBER_UP:
        //     climbUp();
        //     break;
      case STOPPED:
      default:
        handleStopped();
        break;
    }
  }

  /** checks if the subsystems are ready for L1 shooting */
  private boolean subsystemsL1ready() {
    boolean isReady = elevator.atSetPoint();

    return isReady;
  }

  /** checks if the subsystems are ready for L2 shooting */
  private boolean subsystemsL2ready() {
    boolean isReady = elevator.atSetPoint();

    return isReady;
  }

  /** checks if the subsystems are ready for L2 shooting */
  private boolean subsystemsL3ready() {
    boolean isReady = elevator.atSetPoint();

    return isReady;
  }

  /** checks if the subsystems are ready for L2 shooting */
  private boolean subsystemsL4ready() {
    boolean isReady = elevator.atSetPoint();

    return isReady;
  }

  /** moves subsystems to intake position */
  private void intake() {
    elevator.setWantedState(elevatorState.INTAKE);
  }

  /** moves subsystems to L1 position, prepare */
  private void L1prepare() {
    elevator.setWantedState(elevatorState.L1PREPARE);
  }

  /** Shoots Coral to L1 */
  private void L1() {
    elevator.setWantedState(elevatorState.L1);
  }

  /** moves subsystems to L2 position, prepare */
  private void L2prepare() {
    elevator.setWantedState(elevatorState.L2PREPARE);
  }

  /** Shoots Coral to L2 */
  private void L2() {
    elevator.setWantedState(elevatorState.L2);
  }

  /** moves subsystems to L3 position, prepare */
  private void L3prepare() {
    elevator.setWantedState(elevatorState.L3PREPARE);
  }

  /** Shoots Coral to L1 */
  private void L3() {
    elevator.setWantedState(elevatorState.L3);
  }

  /** moves subsystems to L1 position, prepare */
  private void L4prepare() {
    elevator.setWantedState(elevatorState.L4PREPARE);
  }

  /** Shoots Coral to L4 */
  private void L4() {
    elevator.setWantedState(elevatorState.L4);
  }

  /** brings elevator down, stow everything */
  private void stow() {
    elevator.setWantedState(elevatorState.STOW);
  }

  private void handleStopped() {
    drive.stop();
    elevator.stop();
  }

  public BooleanSupplier doesCommandMatch(CurrentSuperState currentSuperState) {
    return () -> Superstructure.currentSuperState == currentSuperState;
  }

  /** State pushers */
  public void setWantedSuperState(WantedSuperState wantedSuperState) {
    this.wantedSuperState = wantedSuperState;
  }

  public Command setWantedSuperStateCommand(WantedSuperState wantedSuperState) {
    return new InstantCommand(() -> setWantedSuperState(wantedSuperState));
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

//     public static enum WantedSuperState {
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

//     public static enum CurrentSuperState {
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

//     private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
//     public static CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
//     private CurrentSuperState previousSuperState;
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
//         currentSuperState = handleStateTransitions();
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

//         // Logger.recordOutput("DesiredSuperstate", wantedSuperState);
//         // if (currentSuperState != previousSuperState) {
//         //     Logger.recordOutput("CurrentSuperstate", currentSuperState);
//         // }

//         // Logger.recordOutput(
//         //         "AimingParameters/AdjustedTurretAngleDegrees",
//         //         aimingParameters.turretAimingAngle().getDegrees());
//         // Logger.recordOutput("AimingParameters/EffectiveDistance",
// aimingParameters.effectiveDistance());

//         // Logger.recordOutput("FeedShotDistance",
// RobotState.getInstance().getDistanceToFeedTarget());
//     }

//     private CurrentSuperState handleStateTransitions() {
//         previousSuperState = currentSuperState;
//         switch (wantedSuperState) {
//             case HOLD_FIX_PIECE:

//                 currentSuperState = CurrentSuperState.HOLD_FIX_PIECE;
//                 break;

//             case REGULAR_STATE:
//                 currentSuperState = CurrentSuperState.REGULAR_STATE;
//                 break;
//             case PREPARING_SUBWOOFER_SHOT:
//                 currentSuperState = CurrentSuperState.PREPARING_SUBWOOFER_SHOT;
//                 break;
//             // case READY_FOR_SUBWOOFER_SHOT:
//             //     currentSuperState = CurrentSuperState.READY_FOR_SUBWOOFER_SHOT;
//             //     break;
//             case SUBWOOFER_SHOT:
//                 currentSuperState = areSystemsReadyForSubwooferShot()
//                         ? CurrentSuperState.SUBWOOFER_SHOT
//                         : CurrentSuperState.PREPARING_SUBWOOFER_SHOT;

//                 break;
//             case PREPARING_LIMELIGHT_SHOT:
//                 currentSuperState = CurrentSuperState.PREPARING_LIMELIGHT_SHOT;
//                 break;
//             // case READY_FOR_LIMELIGHT_SHOT:
//             //     currentSuperState = CurrentSuperState.READY_FOR_LIMELIGHT_SHOT;
//             //     break;
//             case LIMELIGHT_SHOT:
//                 currentSuperState = areSystemsReadyForLimelightShot()
//                 ? CurrentSuperState.LIMELIGHT_SHOT
//                 : CurrentSuperState.PREPARING_LIMELIGHT_SHOT;
//                 break;
//             case PREPARING_PASS:
//                 currentSuperState = CurrentSuperState.PREPARING_PASS;
//                 break;
//             case PASS:

//                 currentSuperState = areSystemsReadyForPassShot() ? CurrentSuperState.PASS :
// CurrentSuperState.PREPARING_PASS;
//                 break;

//             // case READY_FOR_INTAKE:
//             //     currentSuperState = CurrentSuperState.READY_FOR_INTAKE;
//             //     break;
//             case INTAKE_DOWN:
//                 currentSuperState = CurrentSuperState.INTAKE_DOWN;
//                 break;
//             case INTAKE_UP:
//                 currentSuperState = CurrentSuperState.INTAKE_UP;
//                 break;
//             case CLIMBER_UP:
//                 currentSuperState = CurrentSuperState.CLIMBER_UP;
//                 break;
//             case CLIMBER_DOWN:
//                 currentSuperState = CurrentSuperState.CLIMBER_DOWN;
//                 break;

//             case STOPPED:
//             default:
//                 currentSuperState = CurrentSuperState.STOPPED;
//                 break;

//         }
//         return currentSuperState;
//     }

//     private void applyStates() {
//         switch (currentSuperState) {
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
//         wantedSuperState)
//                 && intake.atShootPoint();

//         return isReady;
//     }

//     /** Checks */
//     private boolean areSystemsReadyForLimelightShot() {
//         boolean isReady = flywheel.atSetpoint(drive.calculateShootAngle(),
//         flywheel.getLimelightAndPassRPM(),
//         wantedSuperState)
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
//         wantedSuperState)
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
//         wantedSuperState = WantedSuperState.REGULAR_STATE;
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
//         wantedSuperState = WantedSuperState.REGULAR_STATE;
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
//         wantedSuperState = WantedSuperState.REGULAR_STATE;
//         currentSuperState = CurrentSuperState.REGULAR_STATE;

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

//         wantedSuperState = WantedSuperState.REGULAR_STATE;
//         currentSuperState = CurrentSuperState.REGULAR_STATE;
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

//         wantedSuperState = WantedSuperState.REGULAR_STATE;
//         currentSuperState = CurrentSuperState.REGULAR_STATE;
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
//      * @param currentSuperState that is passed,
//      * {@value} CLIMBER_UP or
//      * {@value} CLIMBER_DOWN
//      */
//     private void climb(){
//         switch (currentSuperState){
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
//     public void setWantedSuperState(WantedSuperState wantedSuperState) {
//         this.wantedSuperState = wantedSuperState;
//     }

//     public Command setWantedSuperStateCommand(WantedSuperState wantedSuperState) {
//         return new InstantCommand(() -> setWantedSuperState(wantedSuperState));
//     }

// }
