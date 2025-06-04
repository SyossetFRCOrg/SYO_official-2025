package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public static enum IntakeState {
    STOPPED,
    INTAKE,
    L1OUTTAKE,
    L2L3OUTTAKE,
    L4OUTTAKE
  }

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private static final HashMap<IntakeState, LoggedTunableNumber> intakeSpeeds = initializeSpeeds();

  // private final Debouncer debouncer = new Debouncer(0.1, DebounceType.kFalling);

  private final Timer debounceTimer = new Timer();
  private final double toleranceTime = 0.1;

  private static final HashMap<IntakeState, LoggedTunableNumber> initializeSpeeds() {
    var map = new HashMap<IntakeState, LoggedTunableNumber>();
    map.put(IntakeState.STOPPED, new LoggedTunableNumber("Intake/StowSpeed", 0));
    map.put(IntakeState.INTAKE, new LoggedTunableNumber("Intake/IntakeSpeed", -5));
    map.put(IntakeState.L1OUTTAKE, new LoggedTunableNumber("Intake/L1Speed", 2));
    map.put(IntakeState.L2L3OUTTAKE, new LoggedTunableNumber("Intake/L2Speed", 4));
    map.put(IntakeState.L4OUTTAKE, new LoggedTunableNumber("Intake/L4Speed", 6));
    return map;
  }

  private static @Getter @Setter IntakeState desiredState = IntakeState.STOPPED;
  private static @Getter @Setter IntakeState currentState = IntakeState.STOPPED;
  private static IntakeState previousState = IntakeState.STOPPED;

  private double intakeSpeed;

  private final IntakeIO intakeIO;

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;

    // new ControllerRumbleCommand(new XboxController(0), () ->
    // debouncer.calculate(inputs.currentAmps > 30));

  }

  @Override
  public void periodic() {

    intakeIO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    IntakeState newState = handleStateTransition();
    if (newState != previousState) {
      Logger.recordOutput("Intake/State", newState.toString());
      previousState = newState;
    }

    applyStates();

    if (inputs.currentAmps < 1.5) {
      debounceTimer.reset();
    }
  }

  public BooleanSupplier intaked() {
    return () -> debounceTimer.get() > toleranceTime;
  }

  private IntakeState handleStateTransition() {
    currentState =
        switch (desiredState) {
          case STOPPED -> IntakeState.STOPPED;
          case INTAKE -> IntakeState.INTAKE;
          case L1OUTTAKE -> IntakeState.L1OUTTAKE;
          case L2L3OUTTAKE -> IntakeState.L2L3OUTTAKE;
          case L4OUTTAKE -> IntakeState.L4OUTTAKE;
          default -> IntakeState.STOPPED;
        };
    return currentState;
  }

  private void applyStates() {
    var state = currentState;

    if (intakeSpeeds.containsKey(state)) {
      intakeSpeed = intakeSpeeds.get(state).get();
    }
    intakeIO.setVelocity(intakeSpeed);
  }

  public void setWantedState(IntakeState state) {
    desiredState = state;
  }
}
