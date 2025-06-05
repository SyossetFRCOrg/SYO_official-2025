package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.util.LoggedTunableNumber;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase implements frc.robot.subsystems.SubsystemSM {

  public enum WantedState {
    STOW,
    INTAKE,
    INTAKELOW,
    L1,
    L2,
    L3,
    L4,
    L2L3ALGAE,
    L3L4ALGAE
  }

  public enum SystemState {
    STOWING,
    INTAKING,
    INTAKINGLOW,
    ATL1,
    ATL2,
    ATL3,
    ATL4,
    ATL2L3ALGAE,
    ATL3L4ALGAE,
    TRANSITIONING
  }

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private final Debouncer atSetpointDebouncer = new Debouncer(0.1);

  private double positionTolerance = 0.03; // radians
  private double velocityTolerance = 0.1; // rad/sec

  // State machine variables
  private WantedState wantedState = WantedState.STOW;
  private SystemState systemState = SystemState.STOWING;

  private static final HashMap<WantedState, LoggedTunableNumber> positions = initializePositions();

  private static final HashMap<WantedState, LoggedTunableNumber> initializePositions() {
    var map = new HashMap<WantedState, LoggedTunableNumber>();

    map.put(WantedState.STOW, new LoggedTunableNumber("Wrist/StowPosition", 0));
    map.put(WantedState.INTAKE, new LoggedTunableNumber("Wrist/IntakePosition", 1.4));
    map.put(WantedState.INTAKELOW, new LoggedTunableNumber("Wrist/IntakeLowPosition", 1.4));
    map.put(WantedState.L1, new LoggedTunableNumber("Wrist/L1Position", 2.2));
    map.put(WantedState.L2, new LoggedTunableNumber("Wrist/L2Position", 3.2));
    map.put(WantedState.L3, new LoggedTunableNumber("Wrist/L3Position", 3.2));
    map.put(WantedState.L4, new LoggedTunableNumber("Wrist/L4Position", 3.58));
    map.put(WantedState.L2L3ALGAE, new LoggedTunableNumber("Wrist/L2L3AlgaePosition", 2.4));
    map.put(WantedState.L3L4ALGAE, new LoggedTunableNumber("Wrist/L3L4AlgaePosition", 2.4));

    return map;
  }

  private double targetPosition = 0;

  public Wrist(WristIO io) {
    this.io = io;
    io.resetPosition(0);
    io.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    // Read inputs
    io.updateInputs(inputs);
    // Log inputs
    Logger.processInputs("Wrist", inputs);

    // Pull Superstate and apply to wrist
    updateFromSuperstructure();

    io.periodic();

    // Logging
    Logger.recordOutput("Wrist/WantedState", wantedState.toString());
    Logger.recordOutput("Wrist/SystemState", systemState.toString());
    Logger.recordOutput("Wrist/AtGoal", atSetPoint());
    Logger.recordOutput("Wrist/TargetPosition", targetPosition);
    Logger.recordOutput("Wrist/CanMove", RobotState.getInstance().isWristCanMove());

    // State machine handling
    handleStateTransitions();
    applyCurrentState();
    System.out.println(targetPosition);
  }

  /**
   * Handles state transitions based on the current system state and wanted state. This method
   * checks if the wrist is at the current setpoint and transitions to the appropriate state if
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
          targetPosition = positions.get(wantedState).get();
        }
        break;
    }
  }

  /**
   * Applies the current system state by moving the wrist to the target position. This method is
   * called in periodic to ensure the wrist moves to the correct position based on the current
   * system state and safety constraints.
   */
  private void applyCurrentState() {
    // Always check if wrist can move - if not, force to stow position
    // if (!RobotState.getInstance().isWristCanMove()) {
    //   targetPosition = positions.get(WantedState.STOW).get();
    //   io.runPosition(targetPosition);
    //   return;
    // }

    switch (systemState) {
      case TRANSITIONING:
        // Move to target position
        io.runPosition(targetPosition);

        break;

      case STOWING:
        targetPosition = positions.get(WantedState.STOW).get();
        io.runPosition(targetPosition);
        break;

      case INTAKING:
        targetPosition = positions.get(WantedState.INTAKE).get();
        io.runPosition(targetPosition);
        break;

      case INTAKINGLOW:
        targetPosition = positions.get(WantedState.INTAKELOW).get();
        io.runPosition(targetPosition);
        break;

      case ATL1:
        targetPosition = positions.get(WantedState.L1).get();
        io.runPosition(targetPosition);
        break;

      case ATL2:
        targetPosition = positions.get(WantedState.L2).get();
        io.runPosition(targetPosition);
        break;

      case ATL3:
        targetPosition = positions.get(WantedState.L3).get();
        io.runPosition(targetPosition);
        break;

      case ATL4:
        targetPosition = positions.get(WantedState.L4).get();
        io.runPosition(targetPosition);
        break;

      case ATL2L3ALGAE:
        targetPosition = positions.get(WantedState.L2L3ALGAE).get();
        io.runPosition(targetPosition);
        break;

      case ATL3L4ALGAE:
        targetPosition = positions.get(WantedState.L3L4ALGAE).get();
        io.runPosition(targetPosition);
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
        MathUtil.isNear(targetPosition, getPosition(), positionTolerance)
            && Math.abs(inputs.velocityRadPerSec) < velocityTolerance);
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
   * Checks if the wrist is at the setpoint for a given SuperState. Maps the SuperState to a
   * WantedState and checks if the wrist is at that setpoint.
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
   * Overloaded: Checks if the wrist is at the setpoint for a given WantedState.
   *
   * @param state
   * @return true if at setpoint, false otherwise.
   */
  public boolean atSetPoint(WantedState state) {
    var position = positions.get(state).get();
    return atSetpointDebouncer.calculate(
        MathUtil.isNear(position, getPosition(), positionTolerance)
            && Math.abs(inputs.velocityRadPerSec) < velocityTolerance);
  }

  public double getPosition() {
    return inputs.positionRad;
  }

  public void resetPosition(double posRads) {
    io.resetPosition(posRads);
  }

  // Integration with existing Superstructure
  public void updateFromSuperstructure() {
    var superState = Superstructure.getCurrentState();
    WantedState newWantedState = mapSuperStateToWantedState(superState);
    if (newWantedState != null) {
      setWantedState(newWantedState);
    }
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
        // Handle prepare states - map to the same positions
      case L1PREPARE:
        return WantedState.L1;
      case L2PREPARE:
        return WantedState.L2;
      case L3PREPARE:
        return WantedState.L3;
      case L4PREPARE:
        return WantedState.L4;
      case INTAKEPREPARE:
        return WantedState.STOW; // Different from original - was INTAKE
      case INTAKELOWPREPARE:
        return WantedState.INTAKE; // Maps to INTAKE position
      default:
        return null; // Don't change state for unmapped states
    }
  }
}
