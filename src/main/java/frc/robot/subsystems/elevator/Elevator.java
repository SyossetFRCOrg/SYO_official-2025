package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.util.LoggedTunableNumber;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase implements frc.robot.subsystems.SubsystemSM {

  public enum WantedState {
    INTAKE,
    INTAKELOW,
    L1,
    L2,
    L3,
    L4,
    STOW,
    L2L3ALGAE,
    L3L4ALGAE
  }

  public enum SystemState {
    INTAKING,
    INTAKINGLOW,
    ATL1,
    ATL2,
    ATL3,
    ATL4,
    STOWING,
    ATL2L3ALGAE,
    ATL3L4ALGAE,
    TRANSITIONING
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Debouncer atSetpointDebouncer = new Debouncer(0.3);

  private double heightTolerance = 1; // radians

  // State machine variables
  private WantedState wantedState = WantedState.STOW;
  private SystemState systemState = SystemState.STOWING;

  private static final HashMap<WantedState, LoggedTunableNumber> heights = initializeHeights();

  private static final HashMap<WantedState, LoggedTunableNumber> initializeHeights() {
    var map = new HashMap<WantedState, LoggedTunableNumber>();

    map.put(WantedState.STOW, new LoggedTunableNumber("Elevator/StowPosition", 24));
    map.put(WantedState.INTAKE, new LoggedTunableNumber("Elevator/IntakePosition", 27.35));
    map.put(WantedState.INTAKELOW, new LoggedTunableNumber("Elevator/LOWIntakePosition", 25.95));
    map.put(WantedState.L1, new LoggedTunableNumber("Elevator/L1Position", 11));
    map.put(WantedState.L2, new LoggedTunableNumber("Elevator/L2Position", 32.6));
    map.put(WantedState.L3, new LoggedTunableNumber("Elevator/L3Position", 47));
    map.put(WantedState.L4, new LoggedTunableNumber("Elevator/L4Position", 70.7));
    map.put(WantedState.L2L3ALGAE, new LoggedTunableNumber("Elevator/L2L3A", 29.7));
    map.put(WantedState.L3L4ALGAE, new LoggedTunableNumber("Elevator/L3L4A", 42));

    return map;
  }

  private double targetHeight = 0;

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    // Read inputs
    io.updateInputs(inputs);
    // Log inputs
    Logger.processInputs("Elevator", inputs);

    // Pull Superstate and apply to elevator

    if (inputs.motorType.equals("Sparkmax")) {
      io.periodic();
    }

    // Logging
    Logger.recordOutput("Elevator/WantedState", wantedState.toString());
    Logger.recordOutput("Elevator/SystemState", systemState.toString());
    Logger.recordOutput("Elevator/AtGoal", atSetPoint());
    Logger.recordOutput("Elevator/TargetHeight", targetHeight);

    // State machine handling
    handleStateTransitions();
    applyCurrentState();
  }

  /**
   * Handles state transitions based on the current system state and wanted state. This method
   * checks if the elevator is at the current setpoint and transitions to the appropriate state if
   * needed.
   */
  private void handleStateTransitions() {
    switch (systemState) {
      case TRANSITIONING:
        if (atCurrentSetpoint()) {
          systemState = getSystemStateFromWanted(wantedState);
        }
        break;
      case STOWING:
      case INTAKING:
      case INTAKINGLOW:
      case ATL1:
      case ATL2:
      case ATL3:
      case ATL4:
      case ATL2L3ALGAE:
      case ATL3L4ALGAE:
        // Check if we need to transition to a new state
        if (wantedState != getWantedStateFromSystem(systemState)) {
          systemState = SystemState.TRANSITIONING;
          targetHeight = heights.get(wantedState).get();
        }
        break;
    }
  }

  /**
   * Applies the current system state by moving the elevator to the target height. This method is
   * called in periodic to ensure the elevator moves to the correct position based on the current
   * system state.
   */
  private void applyCurrentState() {
    switch (systemState) {
      case TRANSITIONING:
        // Move to target height
        io.movetoHeight(targetHeight);
        break;

      case STOWING:
        targetHeight = heights.get(WantedState.STOW).get();
        io.movetoHeight(targetHeight);
        break;

      case INTAKING:
        targetHeight = heights.get(WantedState.INTAKE).get();
        io.movetoHeight(targetHeight);
        break;

      case INTAKINGLOW:
        targetHeight = heights.get(WantedState.INTAKELOW).get();
        io.movetoHeight(targetHeight);
        break;

      case ATL1:
        targetHeight = heights.get(WantedState.L1).get();
        io.movetoHeight(targetHeight);
        break;

      case ATL2:
        targetHeight = heights.get(WantedState.L2).get();
        io.movetoHeight(targetHeight);
        break;

      case ATL3:
        targetHeight = heights.get(WantedState.L3).get();
        io.movetoHeight(targetHeight);
        break;

      case ATL4:
        targetHeight = heights.get(WantedState.L4).get();
        io.movetoHeight(targetHeight);
        break;

      case ATL2L3ALGAE:
        targetHeight = heights.get(WantedState.L2L3ALGAE).get();
        io.movetoHeight(targetHeight);
        break;

      case ATL3L4ALGAE:
        targetHeight = heights.get(WantedState.L3L4ALGAE).get();
        io.movetoHeight(targetHeight);
        break;
    }
  }

  // Helper methods for state machine
  private SystemState getSystemStateFromWanted(WantedState wanted) {
    switch (wanted) {
      case STOW:
        return SystemState.STOWING;
      case INTAKE:
        return SystemState.INTAKING;
      case INTAKELOW:
        return SystemState.INTAKINGLOW;
      case L1:
        return SystemState.ATL1;
      case L2:
        return SystemState.ATL2;
      case L3:
        return SystemState.ATL3;
      case L4:
        return SystemState.ATL4;
      case L2L3ALGAE:
        return SystemState.ATL2L3ALGAE;
      case L3L4ALGAE:
        return SystemState.ATL3L4ALGAE;
      default:
        return SystemState.STOWING;
    }
  }

  private WantedState getWantedStateFromSystem(SystemState system) {
    switch (system) {
      case STOWING:
        return WantedState.STOW;
      case INTAKING:
        return WantedState.INTAKE;
      case INTAKINGLOW:
        return WantedState.INTAKELOW;
      case ATL1:
        return WantedState.L1;
      case ATL2:
        return WantedState.L2;
      case ATL3:
        return WantedState.L3;
      case ATL4:
        return WantedState.L4;
      case ATL2L3ALGAE:
        return WantedState.L2L3ALGAE;
      case ATL3L4ALGAE:
        return WantedState.L3L4ALGAE;
      default:
        return WantedState.STOW;
    }
  }

  private boolean atCurrentSetpoint() {
    return atSetpointDebouncer.calculate(
        MathUtil.isNear(targetHeight, getHeight(), heightTolerance));
  }

  // Public interface methods
  public void setWantedState(WantedState state) {
    wantedState = state;
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  public SystemState getSystemState() {
    return systemState;
  }

  public boolean atSetPoint() {
    return systemState != SystemState.TRANSITIONING && atCurrentSetpoint();
  }

  /**
   * Checks if the elevator is at the setpoint for a given SuperState. Maps the SuperState to a
   * WantedState and checks if the elevator is at that setpoint.
   *
   * @param superState The SuperState to check against.
   * @return true if at setpoint, false otherwise.
   */
  public boolean atSetPoint(SuperState superState) {
    WantedState mappedState = mapSuperStateToWantedState(superState);
    if (mappedState == null) {
      return false; // Unknown state, not at setpoint
    }
    return atSetPoint(mappedState); // run method based on WantedState
  }

  /**
   * Overloaded: Checks if the elevator is at the setpoint for a given WantedState.
   *
   * @param state
   * @return true if at setpoint, false otherwise.
   */
  public boolean atSetPoint(WantedState state) {
    var height = heights.get(state).get();
    return atSetpointDebouncer.calculate(MathUtil.isNear(height, getHeight(), heightTolerance));
  }

  public double getHeight() {
    return inputs.positionRads;
  }

  public void setHeight(double positionRads) {
    io.setHeight(positionRads);
  }

  public void stop() {
    io.stop();
  }

  private WantedState mapSuperStateToWantedState(SuperState superState) {
    switch (superState) {
      case STOW:
        return WantedState.STOW;
      case INTAKE:
        return WantedState.INTAKE;
      case INTAKELOW:
        return WantedState.INTAKELOW;
      case L1:
        return WantedState.L1;
      case L2:
        return WantedState.L2;
      case L3:
        return WantedState.L3;
      case L4:
        return WantedState.L4;
      case L2L3ALGAE:
        return WantedState.L2L3ALGAE;
      case L3L4ALGAE:
        return WantedState.L3L4ALGAE;
        // Handle prepare states - you might want these to map to the same positions
      case L1PREPARE:
        return WantedState.L1;
      case L2PREPARE:
        return WantedState.L2;
      case L3PREPARE:
        return WantedState.L3;
      case L4PREPARE:
        return WantedState.L4;
      case INTAKEPREPARE:
        return WantedState.INTAKE;
      case INTAKELOWPREPARE:
        return WantedState.INTAKELOW;
      default:
        return null; // Don't change state for unmapped states
    }
  }
}
