package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;
import lombok.Getter;
import lombok.Setter;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Debouncer atSetpointDebouncer = new Debouncer(0.3);

  private double heightTolerance = 1; // rad

  DigitalInput zeroLimitSwitch = new DigitalInput(0);

  public enum ElevatorState {
    INTAKE,
    INTAKEPREPARE,
    INTAKELOW,
    INTAKELOWPREPARE,
    L1,
    L1PREPARE,
    L2,
    L2PREPARE,
    L3,
    L3PREPARE,
    L4,
    L4PREPARE,
    STOW,
    STOPPED,
    L2L3ALGAE,
    L3L4ALGAE
  }

  private static @Getter @Setter ElevatorState desiredState = ElevatorState.STOW;
  private static @Getter @Setter ElevatorState currentState = ElevatorState.STOW;
  private static ElevatorState previousState = ElevatorState.STOW;

  private static final HashMap<ElevatorState, LoggedTunableNumber> heights = initializeHeights();

  private static final HashMap<ElevatorState, LoggedTunableNumber> initializeHeights() {
    var map = new HashMap<ElevatorState, LoggedTunableNumber>();
    // to be tuned
    map.put(ElevatorState.STOW, new LoggedTunableNumber("Elevator/StowPosition", 24));
    map.put(ElevatorState.INTAKE, new LoggedTunableNumber("Elevator/IntakePosition", 27.35));
    map.put(ElevatorState.INTAKELOW, new LoggedTunableNumber("Elevator/LOWIntakePosition", 25.95));
    map.put(ElevatorState.L1, new LoggedTunableNumber("Elevator/L1Position", 11));
    map.put(ElevatorState.L2, new LoggedTunableNumber("Elevator/L2Position", 32.6));
    map.put(ElevatorState.L3, new LoggedTunableNumber("Elevator/L3Position", 47));
    map.put(ElevatorState.L4, new LoggedTunableNumber("Elevator/L4Position", 70.7));

    map.put(ElevatorState.L2L3ALGAE, new LoggedTunableNumber("Elevator/L2L3A", 29.7));
    map.put(
        ElevatorState.L3L4ALGAE,
        new LoggedTunableNumber("Elevator/L3L4A", map.get(ElevatorState.L3).get() - 5));

    map.put(ElevatorState.L1PREPARE, map.get(ElevatorState.L1));
    map.put(ElevatorState.L2PREPARE, map.get(ElevatorState.L2));
    map.put(ElevatorState.L3PREPARE, map.get(ElevatorState.L3));
    map.put(ElevatorState.L4PREPARE, map.get(ElevatorState.L4));

    map.put(ElevatorState.INTAKEPREPARE, map.get(ElevatorState.INTAKE));
    map.put(ElevatorState.INTAKELOWPREPARE, map.get(ElevatorState.INTAKELOW));

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

    ElevatorState newState = handleStateTransitions();
    if(newState != previousState) {
      Logger.recordOutput("Elevator/StateChange", newState.toString());
      previousState = newState;
    }

    if (inputs.motorType.equals("Sparkmax")) {
      io.periodic();
    }

    Logger.recordOutput("Elevator/AtGoal", atSetPoint());

    applyStates();

    if (!zeroLimitSwitch.get() && RobotState.getInstance().isLimitSwitching()) {
      io.setHeight(0);
    }
    // System.out.println(zeroLimitSwitch.get());
    Logger.recordOutput("Elevator/LimitSwitch", !zeroLimitSwitch.get());

    // modify the Elevator position in RobotState so that the moduleLimits changes so the max
    // acceleration changes
    // depending on the superstate of the superstructure. can technically do this anywhere, but
    // makes most sense in elevator.

    if (getHeight() >= heights.get(ElevatorState.L4).get() - heightTolerance * 1.3) {
      RobotState.getInstance().setElevatorPosition(4);
    } else if (getHeight() >= heights.get(ElevatorState.L3).get() - heightTolerance * 1.3) {
      RobotState.getInstance().setElevatorPosition(3);
    } else if (getHeight() >= heights.get(ElevatorState.L2).get() - heightTolerance * 1.3) {
      RobotState.getInstance().setElevatorPosition(2);
    } else if (getHeight() >= heights.get(ElevatorState.L1).get() - heightTolerance * 1.3) {
      RobotState.getInstance().setElevatorPosition(1);
    } else if (getHeight() < heights.get(ElevatorState.L1).get() - heightTolerance * 1.3) {
      RobotState.getInstance().setElevatorPosition(0);
    }
   
    RobotState.getInstance()
        .setAboveL1(getHeight() >= heights.get(ElevatorState.L1).get() - heightTolerance);
  }

  private void applyStates() {
    var state = currentState;
    if (heights.containsKey(state)) targetHeight = heights.get(state).get();
    RobotState.getInstance()
        .setWristCanMove(getHeight() > heights.get(ElevatorState.L1).get() - heightTolerance);
    io.movetoHeight(targetHeight);
  }

  private ElevatorState handleStateTransitions(){
    currentState = switch(desiredState){
      case L1 -> ElevatorState.L1;
      case L1PREPARE -> (atSetPoint(ElevatorState.L1) ?  ElevatorState.L1 : ElevatorState.L1PREPARE);
      case L2 -> ElevatorState.L2;
      case L2PREPARE -> (atSetPoint(ElevatorState.L2) ?  ElevatorState.L2 : ElevatorState.L2PREPARE);
      case L3 -> ElevatorState.L3;
      case L3PREPARE -> (atSetPoint(ElevatorState.L3) ?  ElevatorState.L3 : ElevatorState.L3PREPARE);
      case L4 -> ElevatorState.L4;
      case L4PREPARE -> (atSetPoint(ElevatorState.L3) ?  ElevatorState.L4 : ElevatorState.L4PREPARE);
      case INTAKE -> ElevatorState.INTAKE;
      case INTAKEPREPARE -> (atSetPoint(ElevatorState.INTAKE) ?  ElevatorState.INTAKE : ElevatorState.INTAKEPREPARE);
      case INTAKELOW -> ElevatorState.INTAKELOW;
      case INTAKELOWPREPARE -> (atSetPoint(ElevatorState.INTAKELOW) ?  ElevatorState.INTAKELOW : ElevatorState.INTAKELOWPREPARE);
      case STOW -> ElevatorState.STOW;
      case STOPPED -> ElevatorState.STOPPED;
      case L2L3ALGAE -> ElevatorState.L2L3ALGAE;
      case L3L4ALGAE -> ElevatorState.L3L4ALGAE;
      default -> ElevatorState.STOW;
    };
    return currentState;
      
  }

  /** Check if the height is close enough to desired state setpoint */
  public boolean atSetPoint() {
    // Make sure the targetHeight is updated
    return atSetPoint(currentState);
  }

  /** Check if the height is close enough to the given state setpoint */
  public boolean atSetPoint(ElevatorState state) {
    var height = targetHeight;
    if (heights.containsKey(state)) height = heights.get(state).get();
    return atSetpointDebouncer.calculate(MathUtil.isNear(height, getHeight(), heightTolerance));
  }

  public boolean atState(ElevatorState state) {
     return currentState == state;
  }

  /** Returns the current angle of the intake in radians. */
  public double getHeight() {
    return inputs.positionRads;
  }

  // public boolean elevatorUp() {
  //   return getHeight() >= heights.get(SuperState.L2).get();
  // }

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

  public void setWantedState(ElevatorState state) {
    desiredState = state;
  }
}
