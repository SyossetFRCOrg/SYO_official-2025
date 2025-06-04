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
    L2L3ALGAE,
    L3L4ALGAE,
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

  public Substate handleStateTransitions()
  {
    switch(desiredState)
    {
      case INTAKING:
        return Substate.INTAKING;
      case INTAKEPREPARE:
        return Substate.INTAKEPREPARE;
      case L1OUTTAKING:
        return Substate.L1OUTTAKING;
      case L2OUTTAKING:
        return Substate.L2OUTTAKING;
      case L3OUTTAKING:
        return Substate.L3OUTTAKING;
      case L4OUTTAKING:
        return Substate.L4OUTTAKING;
      case OUTTAKEPREPARE:
        return Substate.OUTTAKEPREPARE;
      case INTAKELOW:
        return Substate.INTAKELOW;
      case INTAKELOWPREPARE:
        return Substate.INTAKELOWPREPARE;
      case STOW:
        return Substate.STOW;
      case L2L3ALGAE:
        return Substate.L2L3ALGAE;
      case L3L4ALGAE:
        return Substate.L3L4ALGAE;
      default:
        return null;
    }
  }

  public void applyStates()
  {
    switch(currentState)
    {
      case L1OUTTAKING:
        intakeIO.setVelocity(intakeSpeeds.get(Substate.L1OUTTAKING).get());
        break;
      case L2OUTTAKING:
        intakeIO.setVelocity(intakeSpeeds.get(Substate.L2OUTTAKING).get());
        break;
      case L3OUTTAKING:
        intakeIO.setVelocity(intakeSpeeds.get(Substate.L3OUTTAKING).get());
        break;
      case L4OUTTAKING:
        intakeIO.setVelocity(intakeSpeeds.get(Substate.L4OUTTAKING).get());
        break;
      case INTAKING:
        intakeIO.setVelocity(intakeSpeeds.get(Substate.INTAKING).get());
        break;
      case INTAKELOW:
        intakeIO.setVelocity(intakeSpeeds.get(Substate.INTAKELOW).get());
        break;
      case L2L3ALGAE:
        intakeIO.setVelocity(intakeSpeeds.get(Substate.L2L3ALGAE).get());
        break;
      case L3L4ALGAE:
        intakeIO.setVelocity(intakeSpeeds.get(Substate.L3L4ALGAE).get());
        break;
      default:
        intakeIO.setVelocity(intakeSpeeds.get(currentState).get());
    }
  }

  public BooleanSupplier intaked() {
    return () -> debounceTimer.get() > toleranceTime;
  }
}
