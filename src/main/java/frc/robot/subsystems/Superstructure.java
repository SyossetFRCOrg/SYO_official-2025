package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.Wrist.WristState;

import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;

public class Superstructure extends SubsystemBase {
  private Drive drive;
  private Elevator elevator;
  private RobotContainer container;
  private Wrist wrist;

 

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


  public Superstructure(Drive drive, Elevator elevator, Wrist wrist, RobotContainer container) {
    this.drive = drive;
    this.elevator = elevator;
    this.container = container;
    this.wrist = wrist;
  }

  @Override
  public void periodic() {

    currentState = handleStateTransitions();
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

  }

  /**
   * Sets currentState to the appropiate transition state based on desiredState
   *
   * @return The current super state
   */
  private SuperState handleStateTransitions() {
    previousState = currentState;
    var ready = ready(currentState); 
    currentState =
        switch (desiredState) {
          case L1 ->  ready ? SuperState.L1 : SuperState.L1PREPARE;
          case L2  ->  ready ? SuperState.L2 : SuperState.L2PREPARE;
          case L3 -> ready? SuperState.L3  : SuperState.L3PREPARE;
          case L4   -> ready ? SuperState.L4  : SuperState.L4PREPARE;
          case INTAKE -> ready ? SuperState.INTAKE  : SuperState.INTAKEPREPARE;
          case INTAKELOW -> ready ? SuperState.INTAKELOW  : SuperState.INTAKE;
          default -> currentState;
        };
    return currentState;
  }
  private void applyStates() {
    switch (currentState) {
        case INTAKEPREPARE:
            prepareForIntake();
            break;
        case INTAKE:
            intake();
            break;
        case INTAKELOWPREPARE:
            prepareForLowIntake();
            break;
        case INTAKELOW:
            intakeLow();
            break;
        case L1PREPARE:
            prepareForL1();
            break;
        case L1:
            scoreL1();
            break;
        case L2PREPARE:
            prepareForL2();
            break;
        case L2:
            scoreL2();
            break;
        case L3PREPARE:
            prepareForL3();
            break;
        case L3:
            scoreL3();
            break;
        case L4PREPARE:
            prepareForL4();
            break;
        case L4:
            scoreL4();
            break;
        case STOW:
            stow();
            break;
        case L2L3ALGAE:
            l2l3Algae();
            break;
        case L3L4ALGAE:
            l2l3AlgaePrepare();
            break;
        case STOPPED:
        default:
            handleStopped();
            break;
    }
  }

  private void handleStopped() {
    drive.stop();
    elevator.stop();
  }

  //TODO define the behavior for these methods
  private void prepareForIntake(){
    elevator.setWantedState(ElevatorState.INTAKEPREPARE);
    wrist.setWantedState(WristState.INTAKEPREPARE);
  }
  private void intake(){

  }
  private void prepareForLowIntake(){

  }
  private void intakeLow(){

  }
  private void prepareForL1(){

  }
  private void scoreL1(){

  }
  private void prepareForL2(){

  }
  private void scoreL2(){

  }
  private void prepareForL3(){

  }
  private void scoreL3(){

  }
  private void prepareForL4(){

  }
  private void scoreL4(){

  }
  private void stow(){

  }
  private void l2l3Algae(){

  }
  private void l2l3AlgaePrepare(){

  }


  /** Transition check */
  private boolean ready(SuperState state) {
    return switch (state) {
        // also has to be at alignment goal to score
      case L1 -> elevator.atState(ElevatorState.L1) && wrist.atState(WristState.L1);
      case L2 -> elevator.atState(ElevatorState.L2) && wrist.atState(WristState.L2L3);
      case L3 -> elevator.atState(ElevatorState.L3) && wrist.atState(WristState.L2L3);
      case L4 -> elevator.atState(ElevatorState.L4) && wrist.atState(WristState.L4);
      case INTAKE -> elevator.atState(ElevatorState.INTAKE); //TODO check if &&wrist.atState(WristState.INTAKE);
      case INTAKELOW  -> elevator.atState(ElevatorState.INTAKELOW); //TODO check if &&wrist.atState(WristState.INTAKELOW);
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

