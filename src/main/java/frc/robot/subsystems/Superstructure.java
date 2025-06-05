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
import frc.robot.subsystems.elevator.Elevator.Substate;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;

import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private Drive drive;
  private Elevator elevator;
  private RobotContainer container;
  private Wrist wrist;
  private Intake intake;

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

  private @Getter @Setter  static SuperState desiredSuperState = SuperState.STOW;
  private @Getter @Setter static SuperState currentSuperState = SuperState.STOW;
  private static SuperState previousState = SuperState.STOW;

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
    loggingRobostateValues();
  }

  /**
   * Sets currentState to the appropiate transition state based on desiredState
   *
   * @return The current super state
   */
  private SuperState handleStateTransitions() {
    previousState = currentSuperState;
    var ready = ready(desiredSuperState);
    currentSuperState =
        switch (desiredSuperState) {
          case L1 -> ready ? SuperState.L1 : SuperState.L1PREPARE;
          case L2 -> ready ? SuperState.L2 : SuperState.L2PREPARE;
          case L3 -> ready ? SuperState.L3 : SuperState.L3PREPARE;
          case L4 -> ready ? SuperState.L4 : SuperState.L4PREPARE;
          case INTAKE -> ready ? SuperState.INTAKE : SuperState.INTAKEPREPARE;
          case INTAKELOW -> ready ? SuperState.INTAKELOW : SuperState.INTAKELOWPREPARE;
          default -> desiredSuperState;
        };
    return currentSuperState;
  }

  /**Sets each local subsystem to proper state */
  private void applyStates() {
    switch (currentSuperState) {
      /**For states where intake needs to check Elevator + Wrist, ElevatorWristReady checks if they are at corresponding setpoint */
      case STOPPED:
        elevator.setDesiredState(Elevator.Substate.STOPPED);
        wrist.setDesiredState(Wrist.Substate.STOPPED);
        intake.setDesiredState(Intake.Substate.STOW);
        break;
      case STOW:
        elevator.setDesiredState(Elevator.Substate.STOW);
        wrist.setDesiredState(Wrist.Substate.STOW);
        intake.setDesiredState(Intake.Substate.STOW);
        break;
      case L1, L1PREPARE:
        elevator.setDesiredState(Elevator.Substate.L1);
        wrist.setDesiredState(Wrist.Substate.L1);
        intake.setDesiredState(
          ElevatorWristReady(currentSuperState) ? Intake.Substate.L1OUTTAKING : Intake.Substate.OUTTAKEPREPARE
        );
        break;
      case L2, L2PREPARE:
        elevator.setDesiredState(Elevator.Substate.L2);
        wrist.setDesiredState(Wrist.Substate.L2);
        intake.setDesiredState(
          ElevatorWristReady(currentSuperState) ? Intake.Substate.L2OUTTAKING : Intake.Substate.OUTTAKEPREPARE
        );
        break;
      case L3, L3PREPARE:
        elevator.setDesiredState(Elevator.Substate.L3);
        wrist.setDesiredState(Wrist.Substate.L3);
        intake.setDesiredState(
          ElevatorWristReady(currentSuperState) ? Intake.Substate.L3OUTTAKING : Intake.Substate.OUTTAKEPREPARE
        );
        break;
      case L4, L4PREPARE:
        elevator.setDesiredState(Elevator.Substate.L4);
        wrist.setDesiredState(Wrist.Substate.L4);
        intake.setDesiredState(
          ElevatorWristReady(currentSuperState) ? Intake.Substate.L4OUTTAKING : Intake.Substate.OUTTAKEPREPARE
        );
        break;
      case L2L3ALGAE:
        elevator.setDesiredState(Elevator.Substate.L2L3ALGAE);
        wrist.setDesiredState(Wrist.Substate.L2L3ALGAE);
        intake.setDesiredState(Intake.Substate.L2L3ALGAE);
        break;
      case L3L4ALGAE:
        elevator.setDesiredState(Elevator.Substate.L3L4ALGAE);
        wrist.setDesiredState(Wrist.Substate.L3L4ALGAE);
        intake.setDesiredState(Intake.Substate.L3L4ALGAE);
        break;
      case INTAKELOW, INTAKELOWPREPARE:
        elevator.setDesiredState(Elevator.Substate.INTAKELOW);
        wrist.setDesiredState(Wrist.Substate.INTAKELOW);
        intake.setDesiredState(
            ElevatorWristReady(currentSuperState) ? Intake.Substate.INTAKELOW : Intake.Substate.INTAKELOWPREPARE;
        );
        break;
      case INTAKE, INTAKEPREPARE:
        elevator.setDesiredState(Elevator.Substate.INTAKE);
        wrist.setDesiredState(Wrist.Substate.INTAKE);
        intake.setDesiredState(
            ElevatorWristReady(currentSuperState) ? Intake.Substate.INTAKING : Intake.Substate.INTAKEPREPARE;
        );
    }
  }

  /**@return Whether Elevator, Wrist, and Intake are at the correct state. */
  public boolean ready(SuperState state) {
    return ElevatorWristReady(state) && intakeReady(state);
  }
  /** Transition check */
  private boolean ElevatorWristReady(SuperState state) {
    return switch (state) {
      //return true if both elevator and wrist are in ready state.
      case L1, L1PREPARE ->
        elevator.getCurrentState() == Elevator.Substate.L1 && wrist.getCurrentState() == Wrist.Substate.L1;
      case L2, L2PREPARE ->
        elevator.getCurrentState() == Elevator.Substate.L2 && wrist.getCurrentState() == Wrist.Substate.L2;
      case L3, L3PREPARE ->
        elevator.getCurrentState() == Elevator.Substate.L3 && wrist.getCurrentState() == Wrist.Substate.L3;
      case L4, L4PREPARE ->
        elevator.getCurrentState() == Elevator.Substate.L4 && wrist.getCurrentState() == Wrist.Substate.L4;
      case INTAKE, INTAKEPREPARE ->
        elevator.getCurrentState() == Elevator.Substate.INTAKE && wrist.getCurrentState() == Wrist.Substate.INTAKE;
      case INTAKELOW, INTAKELOWPREPARE ->
        elevator.getCurrentState() == Elevator.Substate.INTAKELOW && wrist.getCurrentState() == Wrist.Substate.INTAKELOW;
      case L2L3ALGAE ->
        elevator.getCurrentState() == Elevator.Substate.L2L3ALGAE && wrist.getCurrentState() == Wrist.Substate.L2L3ALGAE;
      case L3L4ALGAE ->
        elevator.getCurrentState() == Elevator.Substate.L3L4ALGAE && wrist.getCurrentState() == Wrist.Substate.L3L4ALGAE;
      case STOW ->
        elevator.getCurrentState() == Elevator.Substate.STOW && wrist.getCurrentState() == Wrist.Substate.STOW;
      default -> false;
    };
  }

  private boolean intakeReady(SuperState state) {
    switch(state) {
      //if superstructure in prepare state, and intake in prepare state, intake is ready.
      //if superstructure desired is L1, and intake is in L1 prepare state (and previous superstate is L1 prepare), 
      //superstructure can move to L1.
      case L1PREPARE, L2PREPARE, L3PREPARE, L4PREPARE:
        return intake.getCurrentState() == Intake.Substate.OUTTAKEPREPARE;
      case L1:
        return intake.getCurrentState() == Intake.Substate.L1OUTTAKING;
      case L2:
        return intake.getCurrentState() == Intake.Substate.L2OUTTAKING;
      case L3:
        return intake.getCurrentState() == Intake.Substate.L3OUTTAKING;
      case L4:
        return intake.getCurrentState() == Intake.Substate.L4OUTTAKING;
      case INTAKEPREPARE:
        return intake.getCurrentState() == Intake.Substate.INTAKEPREPARE;
      case INTAKE:
        return intake.getCurrentState() == Intake.Substate.INTAKING;
      case INTAKELOWPREPARE:
        return intake.getCurrentState() == Intake.Substate.INTAKELOWPREPARE;
      case INTAKELOW:
        return intake.getCurrentState() == Intake.Substate.INTAKELOW;
      default:
        return false;
    }
  }

  private void handleStopped() {
    drive.stop();
    elevator.stop();
  }

  public BooleanSupplier doesCommandMatch(SuperState currentState) {
    return () -> Superstructure.currentSuperState == currentState;
  }

  /** State pushers */
  public void setWantedSuperState(SuperState desiredState) {
    Superstructure.desiredSuperState = desiredState;
  }

  public Command setWantedSuperStateCommand(SuperState desiredState) {
    return new InstantCommand(
        () -> {
          setWantedSuperState(desiredState);
        });
  }
}
  public void loggingRobostateValues()
  {
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

    Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState.toString());
    Logger.recordOutput("Superstructure/DesiredSuperState", desiredSuperState.toString());

  }