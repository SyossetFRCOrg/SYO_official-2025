package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final Timer debounceTimer = new Timer();
  private final double toleranceTime = 0.1;

  private static final HashMap<SystemState, LoggedTunableNumber> speeds = initializeSpeeds();

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

  private static final HashMap<SystemState, LoggedTunableNumber> initializeSpeeds() {
    var map = new HashMap<SystemState, LoggedTunableNumber>();
    map.put(SystemState.IN_STOW, new LoggedTunableNumber("Intake/StowSpeed", 0));
    map.put(SystemState.INTAKING, new LoggedTunableNumber("Intake/IntakeSpeed", -5));
    map.put(SystemState.L1_ING, new LoggedTunableNumber("Intake/L1Speed", 2));
    map.put(SystemState.L2_ING, new LoggedTunableNumber("Intake/L2Speed", 6));
    map.put(SystemState.L3_ING, new LoggedTunableNumber("Intake/L3Speed", 6));
    map.put(SystemState.L4_ING, new LoggedTunableNumber("Intake/L4Speed", 6));
    map.put(SystemState.L2L3ALGAE_ING, new LoggedTunableNumber("Intake/L2L3AlgaeSpeed", -5));
    map.put(SystemState.L3L4ALGAE_ING, new LoggedTunableNumber("Intake/L3L4AlgaeSpeed", -5));
    map.put(SystemState.INTAKELOW_ING, new LoggedTunableNumber("Intake/IntakeLowSpeed", -5));

    return map;
  }

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    SystemState newState = handleStateTransitions();

    if (newState != systemState) {
      Logger.recordOutput("Intake/SystemState", newState.toString());
      systemState = newState;
    }

    // Handle current management for intake detection
    if (inputs.currentAmps < 1.5) {
      debounceTimer.reset();
    }

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
  }

  public void handleStow() {
    intakeIO.setVelocity(speeds.get(SystemState.IN_STOW).get());
  }

  public void handleL1() {
    intakeIO.setVelocity(speeds.get(SystemState.L1_ING).get());
  }

  public void handleL2() {
    intakeIO.setVelocity(speeds.get(SystemState.L2_ING).get());
  }

  public void handleL3() {
    intakeIO.setVelocity(speeds.get(SystemState.L3_ING).get());
  }

  public void handleL4() {
    intakeIO.setVelocity(speeds.get(SystemState.L4_ING).get());
  }

  public void handleL2L3Algae() {
    intakeIO.setVelocity(speeds.get(SystemState.L2L3ALGAE_ING).get());
  }

  public void handleL3L4Algae() {
    intakeIO.setVelocity(speeds.get(SystemState.L3L4ALGAE_ING).get());
  }

  public void handleIntaking() {
    intakeIO.setVelocity(speeds.get(SystemState.INTAKING).get());
  }

  public void handleIntakeLow() {
    intakeIO.setVelocity(speeds.get(SystemState.INTAKELOW_ING).get());
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

  public BooleanSupplier intaked() {
    return () -> debounceTimer.get() > toleranceTime;
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