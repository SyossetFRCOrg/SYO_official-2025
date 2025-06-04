package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;

import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO wristIO;

  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private final Debouncer aimedDebounce = new Debouncer(.1);

  public enum WristState {
    INTAKE,
    INTAKEPREPARE,
    INTAKELOW,
    INTAKELOWPREPARE,
    L1,
    L1PREPARE,
    L2L3,
    L2L3PREPARE,
    L4,
    L4PREPARE,
    STOW,
    STOPPED,
    L2L3ALGAE,
    L3L4ALGAE
  }

  private static @Getter @Setter WristState desiredState = WristState.STOW;
  private static @Getter @Setter WristState currentState = WristState.STOW;
  private static WristState previousState = WristState.STOW;

  private static final HashMap<WristState, LoggedTunableNumber> positions = initializePositions();

  private static final HashMap<WristState, LoggedTunableNumber> initializePositions() {
    var map = new HashMap<WristState, LoggedTunableNumber>();
    map.put(WristState.STOW, new LoggedTunableNumber("Wrist/StowPosition", 0));
    map.put(WristState.INTAKE, new LoggedTunableNumber("Wrist/IntakePosition", 1.4));

    map.put(WristState.L2L3ALGAE, new LoggedTunableNumber("Wrist/L2L3AlgaePosition", 2.4));
    map.put(WristState.L3L4ALGAE, map.get(WristState.L2L3ALGAE));

    map.put(WristState.L1, new LoggedTunableNumber("Wrist/L1Position", 2.2));
    map.put(WristState.L2L3, new LoggedTunableNumber("Wrist/L2Position", 3.2));
    map.put(WristState.L4, new LoggedTunableNumber("Wrist/L4Position", 3.58));

    map.put(WristState.L1PREPARE, map.get(WristState.L1));
    map.put(WristState.L2L3PREPARE, map.get(WristState.L2L3));
    map.put(WristState.L4PREPARE, map.get(WristState.L4));

    map.put(WristState.INTAKEPREPARE, map.get(WristState.STOW));
    map.put(WristState.INTAKELOW, map.get(WristState.INTAKE));
    map.put(WristState.INTAKELOWPREPARE, map.get(WristState.INTAKE));

    return map;
  }

  private double position = 0;

  public Wrist(WristIO wristIO) {
    this.wristIO = wristIO;
    wristIO.resetPosition(0);
    wristIO.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    wristIO.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    wristIO.periodic();

    WristState newState = handleStateTransitions();
    if (newState != previousState) {
      Logger.recordOutput("Wrist/StateChange", newState.toString());
      previousState = newState;
    }

    applyStates();

  }

  public void resetPosition(double posRads) {
    wristIO.resetPosition(posRads);
  }

  public boolean atSetPoint(WristState setpointState) {

    if (positions.containsKey(setpointState))
      position = positions.get(setpointState).get();
    return aimedDebounce.calculate(
        MathUtil.isNear(position, inputs.positionRad, 0.03 /* 0.106 rad */)
            && Math.abs(inputs.velocityRadPerSec) < .1);
  }

  public double getPosition() {
    return inputs.positionRad;
  }

  private WristState handleStateTransitions() {
    currentState = switch (desiredState) {
      case L1 -> WristState.L1;
      case L1PREPARE -> (atSetPoint(WristState.L1) ? WristState.L1 : WristState.L1PREPARE);
      case L2L3 -> WristState.L2L3;
      case L2L3PREPARE -> (atSetPoint(WristState.L2L3) ? WristState.L2L3 : WristState.L2L3PREPARE);
      case L4 -> WristState.L4;
      case L4PREPARE -> (atSetPoint(WristState.L4) ? WristState.L4 : WristState.L4PREPARE);
      case INTAKE -> (atSetPoint(WristState.INTAKE) ? WristState.INTAKE : WristState.INTAKEPREPARE);
      case INTAKELOW -> (atSetPoint(WristState.INTAKELOW) ? WristState.INTAKELOW : WristState.INTAKELOWPREPARE);
      case STOW -> WristState.STOW;
      case STOPPED -> WristState.STOPPED;
      case L2L3ALGAE -> WristState.L2L3ALGAE;
      case L3L4ALGAE -> WristState.L3L4ALGAE;
      default -> WristState.STOW;
    };

    return currentState;
  }

  private void applyStates() {
    var state = currentState;
    if (positions.containsKey(state) && RobotState.getInstance().isAboveL1()) {
      position = positions.get(state).get();
    } else {
      position = positions.get(WristState.STOW).get();
    }

    // for wrist, everything is in radians
    if (RobotState.getInstance().isWristCanMove()) {
      wristIO.runPosition(position);
    } else {
      wristIO.runPosition(positions.get(WristState.STOW).get());
    }
  }

  public boolean atState(WristState state) {
    return currentState == state;
  }

  public void setWantedState(WristState state) {
    desiredState = state;
  }
}
