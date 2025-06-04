package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.util.LoggedTunableNumber;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;
import lombok.Getter;
import lombok.Setter;

public class Elevator extends SubsystemBase {

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
    L2L3ALGAEPREPARE,
    L3L4ALGAE,
    L3L4ALGAEPREPARE
  }

  private final ElevatorIO io;
  private @Getter Substate currentState;
  private @Setter Substate desiredState;

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Debouncer atSetpointDebouncer = new Debouncer(0.3);

  private double heightTolerance = 1; // rad

  DigitalInput zeroLimitSwitch = new DigitalInput(0);

  private static final HashMap<Substate, LoggedTunableNumber> heights = initializeHeights();

  private static final HashMap<Substate, LoggedTunableNumber> initializeHeights() {
    var map = new HashMap<Substate, LoggedTunableNumber>()
    map.put(Substate.STOW, new LoggedTunableNumber("Elevator/StowPosition", 24));
    map.put(Substate.INTAKE, new LoggedTunableNumber("Elevator/IntakePosition", 27.35));
    map.put(Substate.INTAKELOW, new LoggedTunableNumber("Elevator/LOWIntakePosition", 25.95));
    map.put(Substate.L1, new LoggedTunableNumber("Elevator/L1Position", 11));
    map.put(Substate.L2, new LoggedTunableNumber("Elevator/L2Position", 32.6));
    map.put(Substate.L3, new LoggedTunableNumber("Elevator/L3Position", 47));
    map.put(Substate.L4, new LoggedTunableNumber("Elevator/L4Position", 70.7));

    map.put(Substate.L2L3ALGAE, new LoggedTunableNumber("Elevator/L2L3A", 29.7));
    map.put(
        Substate.L3L4ALGAE,
        new LoggedTunableNumber("Elevator/L3L4A", map.get(SuperState.L3).get() - 5));
    map.put(Substate.L1PREPARE, map.get(Substate.L1));
    map.put(Substate.L2PREPARE, map.get(Substate.L2));
    map.put(Substate.L3PREPARE, map.get(Substate.L3));
    map.put(Substate.L4PREPARE, map.get(Substate.L4));
    return map;
  }

  private double targetHeight = 0;

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.setBrakeMode(true);

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    // io.updateShuffleboard();

    if (inputs.motorType.equals("Sparkmax")) {
      io.periodic();
    }

    // log state transition
    // if (desiredState != currentState) {
    //   Logger.recordOutput("Elevator/Substate", desiredState.toString());
    //   currentState = desiredState;
    // }

    Substate newState = handleStateTransitions();
    if(currentState != newState)
    {
      Logger.recordOutput("Elevator/Substate", newState.toString());
      currentState = newState;
    }
    applyStates();

    Logger.recordOutput("Elevator/AtGoal", atSetPoint());


    if (!zeroLimitSwitch.get() && RobotState.getInstance().isLimitSwitching()) {
      io.setHeight(0);
    }
    // System.out.println(zeroLimitSwitch.get());
    Logger.recordOutput("Elevator/LimitSwitch", !zeroLimitSwitch.get());

    // modify the Elevator position in RobotState so that the moduleLimits changes so the max
    // acceleration changes
    // depending on the superstate of the superstructure. can technically do this anywhere, but
    // makes most sense in elevator.

    if (getHeight() >= heights.get(Substate.L4).get() - heightTolerance * 1.3) {
      RobotState.getInstance().setElevatorPosition(4);
    } else if (getHeight() >= heights.get(Substate.L3).get() - heightTolerance * 1.3) {
      RobotState.getInstance().setElevatorPosition(3);
    } else if (getHeight() >= heights.get(Substate.L2).get() - heightTolerance * 1.3) {
      RobotState.getInstance().setElevatorPosition(2);
    } else if (getHeight() >= heights.get(Substate.L1).get() - heightTolerance * 1.3) {
      RobotState.getInstance().setElevatorPosition(1);
    } else if (getHeight() < heights.get(Substate.L1).get() - heightTolerance * 1.3) {
      RobotState.getInstance().setElevatorPosition(0);
    }

    RobotState.getInstance()
        .setAboveL1(getHeight() >= heights.get(Substate.L1).get() - heightTolerance);
  }

  /** Returns prepare if currently moving towards target state position. Returns target state if at target state position */
  private Substate handleStateTransitions()
  {
    boolean ready = atSetPoint();
    switch(desiredState)
    {
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
        return null;
    }
  }

  private void applyStates() {
    // var state = Superstructure.getCurrentState();
    // if (heights.containsKey(state)) targetHeight = heights.get(state).get();
    // RobotState.getInstance()
    //     .setWristCanMove(getHeight() > heights.get(SuperState.L1).get() - heightTolerance);
    // io.movetoHeight(targetHeight);
    switch (currentState) {
      case STOPPED:
        stop();
        break;
      default:
        moveToDesiredHeight(currentState);
    }
  }


  public void moveToDesiredHeight(Substate state) {
    double desiredHeight = heights.get(state).get();
    io.movetoHeight(desiredHeight);
  }

  // /** Check if the height is close enough to desired state setpoint */
  //   public boolean atSetPoint() {
  //     // Make sure the targetHeight is updated
  //     return atSetPoint(Superstructure.getCurrentSuperState());
  //   }
  public boolean atSetPoint() {
    return atSetPoint(currentState);
  }

  /** Check if the height is close enough to the given state target */
  public boolean atSetPoint(Substate state) {
    double height = heights.get(state).get();
    return atSetpointDebouncer.calculate(MathUtil.isNear(height, getHeight(), heightTolerance));
  }

  /** Returns the current angle of the intake in radians. */
  public double getHeight() {
    return inputs.positionRads;
  }

  /**
   * Resets the angle of the elevator
   *
   * @param positionRads The angle in radians
   */
  public void setHeight(double positionRads) {
    io.setHeight(positionRads);
  }

  /** Stop slam elevator */
  public void stop() {
    io.stop();
  }
}