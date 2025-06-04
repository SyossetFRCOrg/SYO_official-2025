package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.elevator.Elevator.Substate;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import java.lang.annotation.Target;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO wristIO;

  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private final Debouncer aimedDebounce = new Debouncer(.1);

  public enum Substate {
    STOPPED,
    STOW,
    STOWPREPARE,
    INTAKE,
    INTAKEPREPARE,
    INTAKELOW,
    INTAKELOWPREPARE,
    L1PREPARE,
    L2PREPARE,
    L3PREPARE,
    L4PREPARE,
    L1,
    L2,
    L3,
    L4,
    L2L3ALGAE,
    L3L4ALGAE,
    L2L3ALGAEPREPARE,
    L3L4ALGAEPREPARE
  }
  private static final HashMap<Substate, LoggedTunableNumber> positions = initializePositions();
  private @Getter Substate currentState;
  private @Setter Substate desiredState;

  private static final HashMap<Substate, LoggedTunableNumber> initializePositions() {
    var map = new HashMap<Substate, LoggedTunableNumber>();

    map.put(Substate.STOW, new LoggedTunableNumber("Wrist/StowPosition", 0));
    map.put(Substate.STOWPREPARE, map.get(Substate.STOW));

    map.put(Substate.INTAKE, new LoggedTunableNumber("Wrist/IntakePosition", 1.4));
    map.put(Substate.INTAKELOW, new LoggedTunableNumber("Wrist/LOWIntakePosition", 2.4));
    map.put(Substate.INTAKEPREPARE, map.get(Substate.INTAKE));
    map.put(Substate.INTAKELOWPREPARE, map.get(Substate.INTAKELOW));

    map.put(Substate.L1, new LoggedTunableNumber("Wrist/L1Position", 2.2));
    map.put(Substate.L2, new LoggedTunableNumber("Wrist/L2Position", 3.2));
    map.put(Substate.L3, new LoggedTunableNumber("Wrist/L3Position", 3.2));
    map.put(Substate.L4, new LoggedTunableNumber("Wrist/L4Position", 3.58));
    map.put(Substate.L1PREPARE, map.get(Substate.L1));
    map.put(Substate.L2PREPARE, map.get(Substate.L2));
    map.put(Substate.L3PREPARE, map.get(Substate.L3));
    map.put(Substate.L4PREPARE, map.get(Substate.L4));
    map.put(Substate.L2L3ALGAE, new LoggedTunableNumber("Wrist/L2L3A", 2.4));
    map.put(
        Substate.L3L4ALGAE,
        new LoggedTunableNumber("Wrist/L3L4A", map.get(Substate.L2L3ALGAE).get()));
    map.put(Substate.L2L3ALGAEPREPARE, map.get(Substate.L2L3ALGAE));
    map.put(Substate.L3L4ALGAEPREPARE, map.get(Substate.L3L4ALGAE));
    
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

    Substate newState = handleStateTransitions();
    if(newState != currentState) {
      Logger.recordOutput("Wrist/Substate", newState.toString());
      currentState = newState;
    }
    // if (positions.containsKey(Superstructure.getCurrentSuperState())
    //     && RobotState.getInstance().isAboveL1()) {
    //   position = positions.get(Superstructure.getCurrentSuperState()).get();
    // } else {
    //   position = positions.get(SuperState.STOW).get();
    // }
    if(desiredState != currentState) {
      Logger.recordOutput("Wrist/Substate", desiredState.toString());
      currentState = desiredState;
    }
    applyStates();
    // for wrist, everything is in radians
    if (RobotState.getInstance().isWristCanMove()) wristIO.runPosition(position);
    else wristIO.runPosition(positions.get(Substate.STOW).get());
  }

  private Substate handleStateTransitions() {
    boolean ready = atSetPoint();
    switch(desiredState) {
      case STOPPED:
        return Substate.STOPPED;
      case L1, L1PREPARE:
        return ready ? Substate.L1 : Substate.L1PREPARE;
      case L2, L2PREPARE:
        return ready ? Substate.L2 : Substate.L2PREPARE;
      case L3, L3PREPARE:
        return ready ? Substate.L3 : Substate.L3PREPARE;
      case L4, L4PREPARE:
        return ready ? Substate.L4 : Substate.L4PREPARE;
      case INTAKE, INTAKEPREPARE:
        return ready ? Substate.INTAKE : Substate.INTAKEPREPARE;
      case INTAKELOW, INTAKELOWPREPARE:
        return ready ? Substate.INTAKELOW : Substate.INTAKELOWPREPARE;
      case STOW, STOWPREPARE:
        return ready ? Substate.STOW : Substate.STOWPREPARE;
      case L2L3ALGAE, L2L3ALGAEPREPARE:
        return ready ? Substate.L2L3ALGAE : Substate.L2L3ALGAEPREPARE;
      case L3L4ALGAE, L3L4ALGAEPREPARE:
        return ready ? Substate.L3L4ALGAE : Substate.L3L4ALGAEPREPARE;
      default:
        return Substate.STOW;
    }
  }

  private void applyStates() {
    switch(currentState) {
      case STOPPED:
        wristIO.stop();
      default:
        moveToDesiredPosition(currentState);
    }
  }

  /** Moves wrist to target position */
  public void moveToDesiredPosition(Substate state) {
    double desiredPosition = positions.get(state).get();
    wristIO.runPosition(desiredPosition);
  }

  public void resetPosition(double posRads) {
    wristIO.resetPosition(posRads);
  }

  /** checks if wrist is at setpoint */
  public boolean atSetPoint() {
    return atSetPoint(currentState);
  }
  /** checks if wrist is at setpoint given a target */
  public boolean atSetPoint(Substate state) {
    double position = positions.get(state).get();
    return aimedDebounce.calculate(MathUtil.isNear(position, inputs.positionRad, 0.03 /*0.106 rad*/)
               && Math.abs(inputs.velocityRadPerSec) < .1);
  }

  public double getPosition() {
    return inputs.positionRad;
  }
}
