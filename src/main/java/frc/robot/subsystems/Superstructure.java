package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.intake.Intake;

// Removed invalid import statement
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private Drive drive;
  private Elevator elevator;
  private RobotContainer container;
  private Wrist wrist;
  private Intake intake; // implement

  //do something
  private boolean lowIntake = false;

  public static enum WantedSuperState {
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
    STOP,
    STOW,
    L2L3ALGAE,
    L3L4ALGAE
  }

  // Change into present tense
  public static enum CurrentSuperState {
    // MANUAL,
    INTAKEPREPARED,
    INTAKED,
    INTAKELOW_ED,
    INTAKELOWPREPARED,
    L1_ED,
    L2_ED,
    L3_ED,
    L4_ED,
    L1PREPARED,
    L2PREPARED,
    L3PREPARED,
    L4PREPARED,
    STOPPED,
    STOWED,
    L2L3ALGAE_ED,
    L3L4ALGAE_ED
    }

  private WantedSuperState wantedSuperState = WantedSuperState.STOW;
  private CurrentSuperState currentSuperState = CurrentSuperState.STOWED;
  private static CurrentSuperState previousState = CurrentSuperState.STOWED;

  public Superstructure(Drive drive, Elevator elevator, Wrist wrist, RobotContainer container, Intake intake) {
    this.drive = drive;
    this.elevator = elevator;
    this.container = container;
    this.wrist = wrist;
    this.intake = intake;
  }

  @Override
  public void periodic() {
    currentSuperState = handleStateTransitions();
    applyStates();

    // janky way of logging robotstate values. Robot state @AutoLog doesn't work???
    Logger.recordOutput("RobotState/aboveL1", RobotState.getInstance().isAboveL1());

    Logger.recordOutput(
        "RobotState/elevatorPosition", RobotState.getInstance().getElevatorPosition());

    Logger.recordOutput("RobotState/addingVision", RobotState.getInstance().isAddingVision());

    Logger.recordOutput("RobotState/wristCanMove", RobotState.getInstance().isWristCanMove());

    Logger.recordOutput(
        "RobotState/reefAutoAligning", RobotState.getInstance().isReefAutoAligning());

    Logger.recordOutput("RobotState/reefAutoAiming", RobotState.getInstance().isReefAutoAiming());

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

    Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState.toString());
    Logger.recordOutput("Superstructure/DesiredSuperState", wantedSuperState.toString());

    if (currentSuperState == CurrentSuperState.STOPPED) handleStopped();
  }

  /**
   * Sets currentState to the appropiate transition state based on desiredState
   *
   * @return The current super state
   */
  private CurrentSuperState handleStateTransitions() {
    previousState = currentSuperState;
    currentSuperState =
        switch (wantedSuperState) {
          case L1 -> isReadyForL1() ? CurrentSuperState.L1_ED : CurrentSuperState.L1PREPARED;
          case L2 -> isReadyForL2() ? CurrentSuperState.L2_ED : CurrentSuperState.L2PREPARED;
          case L3 -> isReadyForL3() ? CurrentSuperState.L3_ED : CurrentSuperState.L3PREPARED;
          case L4 -> isReadyForL4() ? CurrentSuperState.L4_ED : CurrentSuperState.L4PREPARED;
          case INTAKE -> isReadyForIntake() ? CurrentSuperState.INTAKED : CurrentSuperState.INTAKEPREPARED;
          case INTAKELOW -> isReadyForLowIntake() ? CurrentSuperState.INTAKELOW_ED : CurrentSuperState.INTAKELOWPREPARED;
          case INTAKEPREPARE -> CurrentSuperState.INTAKEPREPARED;
          case INTAKELOWPREPARE -> CurrentSuperState.INTAKELOWPREPARED;
          case STOW -> CurrentSuperState.STOWED;
          case L2L3ALGAE -> CurrentSuperState.L2L3ALGAE_ED;
          case L3L4ALGAE -> CurrentSuperState.L3L4ALGAE_ED;
          case L1PREPARE -> CurrentSuperState.L1PREPARED;
          case L2PREPARE -> CurrentSuperState.L2PREPARED;
          case L3PREPARE -> CurrentSuperState.L3PREPARED;
          case L4PREPARE -> CurrentSuperState.L4PREPARED;
          case STOP -> {
            handleStopped();
            yield CurrentSuperState.STOPPED;
          }
          default -> {
            handleStopped();
            yield CurrentSuperState.STOPPED;
          }
        };

    return currentSuperState;
  }

  /** Applies the current super state to the subsystems */
  private void applyStates() {
    switch (currentSuperState) {
      case INTAKEPREPARED -> handleIntakePrepare();
      case INTAKED -> handleIntake();
      case INTAKELOW_ED -> handleIntakeLow();
      case INTAKELOWPREPARED -> handleIntakeLowPrepare();
      case L1_ED -> handleL1();
      case L2_ED -> handleL2();
      case L3_ED -> handleL3();
      case L4_ED -> handleL4();
      case L1PREPARED -> handleL1Prepare();
      case L2PREPARED -> handleL2Prepare();
      case L3PREPARED -> handleL3Prepare();
      case L4PREPARED -> handleL4Prepare();
      case STOPPED -> handleStop();
      case STOWED -> handleStow();
      case L2L3ALGAE_ED -> handleL2L3Algae();
      case L3L4ALGAE_ED -> handleL3L4Algae();
    }
  }

  /** Transition check */
  private boolean isReadyForL2() {
    return elevator.atSetPoint()
        && wrist.atL2()
        && container.getReefAlignController().atGoal();
  }
  private boolean isReadyForL3() {
    return elevator.atSetPoint()
        && wrist.atL3()
        && container.getReefAlignController().atGoal();
  }
  private boolean isReadyForL4() {
    return elevator.atSetPoint()
        && wrist.atL4()
        && container.getReefAlignController().atGoal();
  }

  private boolean isReadyForL1() {
    return elevator.atSetPoint() && wrist.atL1();
  }

  private boolean isReadyForIntake() {
    return elevator.atSetPoint();
  }

  private boolean isReadyForLowIntake() {
    return elevator.atSetPoint();
  }

  // public BooleanSupplier doesCommandMatch(CurrentSuperState currentState) {
  //   return () -> this.currentSuperState == currentState;
  // }

  /** Subsystem states */
  private void handleIntakePrepare() {
    wrist.setWantedState(Wrist.WantedState.STOW);
  }

  private void handleIntake() {
    wrist.setWantedState(Wrist.WantedState.STOW);
  }

  private void handleIntakeLow() {
    wrist.setWantedState(Wrist.WantedState.INTAKE);
  }

  private void handleIntakeLowPrepare() {
    wrist.setWantedState(Wrist.WantedState.INTAKE);
  }

  private void handleL1() {
    wrist.setWantedState(Wrist.WantedState.L1);
  }

  private void handleL2() {
    wrist.setWantedState(Wrist.WantedState.L2);
  }

  private void handleL3() {
    wrist.setWantedState(Wrist.WantedState.L3);
  }

  private void handleL4() {
    wrist.setWantedState(Wrist.WantedState.L4);
  }

  private void handleL1Prepare() {
    wrist.setWantedState(Wrist.WantedState.L1);
  }

  private void handleL2Prepare() {
    wrist.setWantedState(Wrist.WantedState.L2);
  }

  private void handleL3Prepare() {
    wrist.setWantedState(Wrist.WantedState.L3);
  }

  private void handleL4Prepare() {
    wrist.setWantedState(Wrist.WantedState.L4);
  }

  private void handleStop() {
    wrist.setWantedState(Wrist.WantedState.STOW);
  }

  private void handleStow() {
    wrist.setWantedState(Wrist.WantedState.STOW);
  }

  private void handleL2L3Algae() {
    wrist.setWantedState(Wrist.WantedState.L2L3ALGAE);
  }

  private void handleL3L4Algae() {
    wrist.setWantedState(Wrist.WantedState.L3L4ALGAE);
  }

  /** Utility states */
  private void handleStopped() {
    drive.stop();
    elevator.stop();
  }

  /** State pushers */
  public void setWantedSuperState(WantedSuperState wantedSuperState) {
    this.wantedSuperState = wantedSuperState;
  }

  public Command setWantedSuperStateCommand(WantedSuperState wantedSuperState) {
    return new InstantCommand(
        () -> {
          setWantedSuperState(wantedSuperState);
        });
  }
  
  public WantedSuperState getWantedSuperState() {
    return wantedSuperState;
  }

  public CurrentSuperState getCurrentSuperState() {
    return currentSuperState;
  }
}
