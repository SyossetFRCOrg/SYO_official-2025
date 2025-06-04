package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.util.LoggedTunableNumber;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import javax.lang.model.util.ElementScanner14;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public static enum Substate {
    // INTAKE_IN,
    // INTAKE_OUT,
    INTAKEPREPARE,
    OUTTAKEPREPARE,
    INTAKING,
    L1OUTTAKING,
    L2OUTTAKING,
    L3OUTTAKING,
    L4OUTTAKING,
    INTAKELOW,
    INTAKELOWPREPARE,
    STOW,
    L2L3ALGAE, //Algae don't have prepare state because we want rollers to be rolling whole time
    L3L4ALGAE
  }
  

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private static final HashMap<Substate, LoggedTunableNumber> intakeSpeeds = initializeSpeeds();

  // private final Debouncer debouncer = new Debouncer(0.1, DebounceType.kFalling);

  private final Timer debounceTimer = new Timer();
  private final double toleranceTime = 0.1;

  private static final HashMap<Substate, LoggedTunableNumber> initializeSpeeds() {
    var map = new HashMap<Substate, LoggedTunableNumber>();

    map.put(Substate.STOW, new LoggedTunableNumber("Intake/StowSpeed", 0));

    //TO BE TUNED ?
    map.put(Substate.INTAKING, new LoggedTunableNumber("Intake/IntakeSpeed", -5)); 
    map.put(Substate.L1OUTTAKING, new LoggedTunableNumber("Intake/L1_OuttakeSpeed", 2)); 
    map.put(Substate.L2OUTTAKING, new LoggedTunableNumber("Intake/L2_OuttakeSpeed", 4));
    map.put(Substate.L3OUTTAKING, new LoggedTunableNumber("Intake/L3_OuttakeSpeed", 4));
    map.put(Substate.L4OUTTAKING, new LoggedTunableNumber("Intake/L4_OuttakeSpeed", 6));
    map.put(Substate.INTAKELOW, map.get(Substate.INTAKING));
    map.put(Substate.INTAKELOWPREPARE, map.get(Substate.STOW));
    map.put(Substate.OUTTAKEPREPARE, map.get(Substate.STOW));
    map.put(Substate.INTAKEPREPARE, map.get(Substate.STOW));
    map.put(Substate.L2L3ALGAE, new LoggedTunableNumber("Intake/L2L3AlgaeSpeed", -5));
    map.put(Substate.L3L4ALGAE, new LoggedTunableNumber("Intake/L3L4AlgaeSpeed", -5));
    return map;
  }

  public @Getter @Setter Substate currentState = Substate.STOW;
  public @Getter @Setter Substate desiredState = Substate.STOW;

  private double intakeSpeed;
  private final IntakeIO intakeIO;

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    //properly transitions states
    Substate newState = handleStateTransitions();
    if(newState != currentState)
    {
      Logger.recordOutput("Intake/Substate", newState.toString());
      currentState = newState;
    }
    applyStates();
    intakeIO.setVelocity(intakeSpeed);

    if (inputs.currentAmps < 1.5) {
      debounceTimer.reset();
    }
  }

  /**
   * 
   * @return desired state
   */
  public Substate handleStateTransitions() {
    switch(desiredState) {
      case INTAKING:
      case INTAKEPREPARE:
      case L1OUTTAKING:
      case L2OUTTAKING:
      case L3OUTTAKING:
      case L4OUTTAKING:
      case OUTTAKEPREPARE:
      case INTAKELOW:
      case INTAKELOWPREPARE:
      case STOW:
      case L2L3ALGAE:
      case L3L4ALGAE:
      //handle state transitions pretty pointless in intake tbh
        return desiredState;
      default:
        return null;
    }
  }

  /**Sets intake velocity according to state */
  public void applyStates()
  {
    switch(currentState)
    {
      case L1OUTTAKING:
      case L2OUTTAKING:
      case L3OUTTAKING:
      case L4OUTTAKING:
      case INTAKING:
      case INTAKELOW:
      case L2L3ALGAE:
      case L3L4ALGAE:
      default:
        intakeIO.setVelocity(intakeSpeeds.get(currentState).get());
    }
  }

  public BooleanSupplier intaked() {
    return () -> debounceTimer.get() > toleranceTime;
  }
}
