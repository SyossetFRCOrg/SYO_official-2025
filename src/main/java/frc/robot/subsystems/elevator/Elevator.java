package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Debouncer atSetpointDebouncer = new Debouncer(0.5);

  private double heightTolerance = .8; // rad

  private static final HashMap<SystemState, LoggedTunableNumber> heights = initializeHeights();

  public enum WantedState {
    STOW,
    L1,
    L2,
    L3,
    L4,
    L2L3ALGAE,
    L3L4ALGAE,
    INTAKE,
    INTAKELOW
  }

  public enum SystemState {
    IN_STOW,
    L1_ING,
    L2_ING,
    L3_ING,
    L4_ING,
    L2L3ALGAE_ING,
    L3L4ALGAE_ING,
    INTAKING,
    INTAKELOW_ING
  }

  private WantedState wantedState = WantedState.STOW;
  private SystemState systemState = SystemState.IN_STOW;

  private static final HashMap<SystemState, LoggedTunableNumber> initializeHeights() {
    var map = new HashMap<SystemState, LoggedTunableNumber>();
    map.put(SystemState.IN_STOW, new LoggedTunableNumber("Elevator/StowPosition", 0));
    map.put(SystemState.INTAKING, new LoggedTunableNumber("Elevator/IntakePosition", 30));
    map.put(SystemState.INTAKELOW_ING, new LoggedTunableNumber("Elevator/LOWIntakePosition", 26.5));
    map.put(SystemState.L1_ING, new LoggedTunableNumber("Elevator/L1Position", 11));
    map.put(SystemState.L2_ING, new LoggedTunableNumber("Elevator/L2Position", 36.4));
    map.put(SystemState.L3_ING, new LoggedTunableNumber("Elevator/L3Position", 51));
    map.put(SystemState.L4_ING, new LoggedTunableNumber("Elevator/L4Position", 74.5));

    map.put(SystemState.L2L3ALGAE_ING, map.get(SystemState.L2_ING));
    map.put(SystemState.L3L4ALGAE_ING, 
        new LoggedTunableNumber("Elevator/L3L4A", map.get(SystemState.L3_ING).get() - 5));

    return map;
  }

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    io.updateShuffleboard();

    if (inputs.motorType.equals("Sparkmax")) {
      io.periodic();
    }

    SystemState newState = handleStateTransitions();

    if (newState != systemState) {
      Logger.recordOutput("Elevator/SystemState", newState.toString());
      systemState = newState;
    }

    Logger.recordOutput("Elevator/AtGoal", atSetPoint());

    // Execute state-specific behavior
    switch (systemState) {
      case IN_STOW -> handleStow();
      case L1_ING -> handleL1();
      case L2_ING -> handleL2();
      case L3_ING -> handleL3();
      case L4_ING -> handleL4();
      case L2L3ALGAE_ING -> handleL2L3Algae();
      case L3L4ALGAE_ING -> handleL3L4Algae();
      case INTAKING -> handleIntaking();
      case INTAKELOW_ING -> handleIntakeLow();
      default -> handleStow();
    }

    // Update RobotState based on current height
    updateRobotState();
  }

  public void handleStow() {
    io.movetoHeight(heights.get(SystemState.IN_STOW).get());
  }

  public void handleL1() {
    io.movetoHeight(heights.get(SystemState.L1_ING).get());
  }

  public void handleL2() {
    io.movetoHeight(heights.get(SystemState.L2_ING).get());
  }

  public void handleL3() {
    io.movetoHeight(heights.get(SystemState.L3_ING).get());
  }

  public void handleL4() {
    io.movetoHeight(heights.get(SystemState.L4_ING).get());
  }

  public void handleL2L3Algae() {
    io.movetoHeight(heights.get(SystemState.L2L3ALGAE_ING).get());
  }

  public void handleL3L4Algae() {
    io.movetoHeight(heights.get(SystemState.L3L4ALGAE_ING).get());
  }

  public void handleIntaking() {
    io.movetoHeight(heights.get(SystemState.INTAKING).get());
  }

  public void handleIntakeLow() {
    io.movetoHeight(heights.get(SystemState.INTAKELOW_ING).get());
  }

  private SystemState handleStateTransitions() {
    return switch (wantedState) {
      case STOW -> SystemState.IN_STOW;
      case L1 -> SystemState.L1_ING;
      case L2 -> SystemState.L2_ING;
      case L3 -> SystemState.L3_ING;
      case L4 -> SystemState.L4_ING;
      case L2L3ALGAE -> SystemState.L2L3ALGAE_ING;
      case L3L4ALGAE -> SystemState.L3L4ALGAE_ING;
      case INTAKE -> SystemState.INTAKING;
      case INTAKELOW -> SystemState.INTAKELOW_ING;
      default -> SystemState.IN_STOW;
    };
  }

  private void updateRobotState() {
    // Update elevator position for drive limits
    if (getHeight() >= heights.get(SystemState.L4_ING).get() - heightTolerance * 1.3) {
      RobotState.getInstance().setElevatorPosition(4);
    } else if (getHeight() >= heights.get(SystemState.L3_ING).get() - heightTolerance * 1.3) {
      RobotState.getInstance().setElevatorPosition(3);
    } else if (getHeight() >= heights.get(SystemState.L2_ING).get() - heightTolerance * 1.3) {
      RobotState.getInstance().setElevatorPosition(2);
    } else if (getHeight() >= heights.get(SystemState.L1_ING).get() - heightTolerance * 1.3) {
      RobotState.getInstance().setElevatorPosition(1);
    } else {
      RobotState.getInstance().setElevatorPosition(0);
    }

    // Update wrist movement permission
    RobotState.getInstance()
        .setWristCanMove(getHeight() > heights.get(SystemState.L1_ING).get() - heightTolerance);

    // Update above L1 status
    RobotState.getInstance()
        .setAboveL1(getHeight() >= heights.get(SystemState.L1_ING).get() - heightTolerance);
  }

  /** Check if the height is close enough to current state setpoint */
  public boolean atSetPoint() {
    var targetHeight = heights.get(systemState).get();
    return atSetpointDebouncer.calculate(MathUtil.isNear(targetHeight, getHeight(), heightTolerance));
  }

  /** Check if the height is close enough to the given state setpoint */
  public boolean atSetPoint(SystemState state) {
    var height = heights.get(state).get();
    return atSetpointDebouncer.calculate(MathUtil.isNear(height, getHeight(), heightTolerance));
  }

  /** Returns the current height of the elevator in radians. */
  public double getHeight() {
    return inputs.positionRads;
  }

  /**
   * Resets the height of the elevator
   *
   * @param positionRads The height in radians
   */
  public void setHeight(double positionRads) {
    io.setHeight(positionRads);
  }

  /** Stop elevator */
  public void stop() {
    io.stop();
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  public SystemState getSystemState() {
    return systemState;
  }
}