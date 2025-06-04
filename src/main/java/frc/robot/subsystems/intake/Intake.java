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
    STOPPED,
    INTAKE_PREPARE,
    OUTTAKE_PREPARE,
    INTAKING,
    L1_OUTTAKING,
    L2_OUTTAKING,
    L3_OUTTAKING,
    L4_OUTTAKING,
    INTAKE_LOW,
    INTAKE_LOW_PREPARE,
    STOW
  }
  

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private static final HashMap<SuperState, LoggedTunableNumber> intakeSpeeds = initializeSpeeds();

  // private final Debouncer debouncer = new Debouncer(0.1, DebounceType.kFalling);

  private final Timer debounceTimer = new Timer();
  private final double toleranceTime = 0.1;

  private static final HashMap<Substate, LoggedTunableNumber> initializeSpeeds() {
    var map = new HashMap<Substate, LoggedTunableNumber>();
    // map.put(SuperState.STOW, new LoggedTunableNumber("Intake/StowSpeed", 0));
    // map.put(SuperState.INTAKE, new LoggedTunableNumber("Intake/IntakeSpeed", -5));
    // map.put(SuperState.L1, new LoggedTunableNumber("Intake/L1Speed", 2));
    // map.put(SuperState.L2, new LoggedTunableNumber("Intake/L2Speed", 4));
    // map.put(SuperState.L3, new LoggedTunableNumber("Intake/L3Speed", 4));
    // map.put(SuperState.L4, new LoggedTunableNumber("Intake/L4Speed", 6));

    // map.put(SuperState.L1PREPARE, map.get(SuperState.STOW));
    // map.put(SuperState.L2PREPARE, map.get(SuperState.STOW));
    // map.put(SuperState.L3PREPARE, map.get(SuperState.STOW));
    // map.put(SuperState.L4PREPARE, map.get(SuperState.STOW));

    // map.put(SuperState.INTAKEPREPARE, map.get(SuperState.STOW));
    // map.put(SuperState.INTAKELOWPREPARE, map.get(SuperState.STOW));

    // map.put(SuperState.L2L3ALGAE, map.get(SuperState.INTAKE));
    // map.put(SuperState.L3L4ALGAE, map.get(SuperState.INTAKE));

    // map.put(SuperState.INTAKELOW, map.get(SuperState.INTAKE));

    map.put(Substate.STOW, new LoggedTunableNumber("Intake/StowSpeed", 0));
    map.put(Substate.INTAKING, new LoggedTunableNumber("Intake/IntakeSpeed", 2)); //TO BE TUNED
    map.put(Substate.OUTTAKING, new LoggedTunableNumber("Outtake/OuttakeSpeed", -2)); //TO BE TUNED

    //Confused on why L1 L2 L3 L4 have different intake speeds.
    // map.put(SuperState.L1, new LoggedTunableNumber("Intake/L1Speed", 2));
    // map.put(SuperState.L2, new LoggedTunableNumber("Intake/L2Speed", 4));
    // map.put(SuperState.L3, new LoggedTunableNumber("Intake/L3Speed", 4));
    // map.put(SuperState.L4, new LoggedTunableNumber("Intake/L4Speed", 6));

    // map.put(SuperState.L1PREPARE, map.get(SuperState.STOW));
    // map.put(SuperState.L2PREPARE, map.get(SuperState.STOW));
    // map.put(SuperState.L3PREPARE, map.get(SuperState.STOW));
    // map.put(SuperState.L4PREPARE, map.get(SuperState.STOW));

    map.put(Substate.INTAKE_PREPARE, map.get(Substate.STOW));
    map.put(Substate.INTAKE_LOW_PREPARE, map.get(Substate.STOW));

    // map.put(Substate.L2L3ALGAE, map.get(Substate.INTAKE));
    // map.put(Substate.L3L4ALGAE, map.get(Substate.INTAKE));
    map.put(Substate.INTAKE_LOW, map.get(Substate.INTAKING));
    

    return map;
  }

  // private final LoggedTunableNumber intakeVelocity =
  //     new LoggedTunableNumber("Intake/IntakeVelocity", 2);
  // private final LoggedTunableNumber outtakeVelocity =
  //     new LoggedTunableNumber("Intake/OuttakeVelocity", -2);

  public @Getter @Setter Substate currentState = Substate.STOW;
  public @Getter @Setter Substate desiredState = Substate.STOW;

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

    // if (intakeSpeeds.containsKey(Superstructure.getCurrentSuperState())) {
    //   intakeSpeed = intakeSpeeds.get(Superstructure.getCurrentSuperState()).get();
    // }

    Substate newState = handleStateTransitions();
    if(newState != currentState)
    {
      Logger.recordOutput("Intake/Substate", newState.toString());
      currentState = newState;
    }

    intakeIO.setVelocity(intakeSpeed);

    if (inputs.currentAmps < 1.5) {
      debounceTimer.reset();
    }
    // intakeSpeed * 6
    // switch (state) {
    //   case INTAKING:

    //     intakeIO.setVelocity(intakeVelocity.get());
    //     break;
    //   case OUTTAKING:
    //     intakeIO.setVelocity(outtakeVelocity.get());
    //     break;
    //   case STOW:
    //   default:
    //     intakeIO.setVelocity(0);
    //     break;
    // }
  }

  public Substate handleStateTransitions()
  {
    switch(desiredState)
    {
      case STOPPED:
        //stop logic here
        return Substate.STOPPED;
      case INTAKING:
        return Substate.INTAKING;
      case INTAKE_PREPARE:
        return Substate.INTAKE_PREPARE;
      case OUTTAKING:
        return Substate.OUTTAKING;
      case OUTTAKE_PREPARE:
        return Substate.OUTTAKE_PREPARE;
      case STOW:
        return Substate.STOW;
      default:
        return null;
    }
  }

  public void applyStates()
  {
    switch(currentState)
    {
      case INTAKE_PREPARE:
        break;
      case OUTTAKE_PREPARE:
        break;
      case INTAKING:
        intakeIO.setVelocity(intakeSpeeds.get(SuperState.INTAKE).get());
        break;
      case OUTTAKING:
        intakeIO.setVelocity(intakeSpeeds.get(SuperState.INTAKE).get());
        break;
    }
  }

  public BooleanSupplier intaked() {
    return () -> debounceTimer.get() > toleranceTime;
  }


}
