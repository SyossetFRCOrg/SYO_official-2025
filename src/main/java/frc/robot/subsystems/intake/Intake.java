package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.util.LoggedTunableNumber;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public static enum SubState {
    INTAKE_IN,
    INTAKE_OUT,
    INTAKING,
    OUTTAKING,
    STOW,
  }

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private static final HashMap<SuperState, LoggedTunableNumber> intakeSpeeds = initializeSpeeds();

  // private final Debouncer debouncer = new Debouncer(0.1, DebounceType.kFalling);

  private final Timer debounceTimer = new Timer();
  private final double toleranceTime = 0.1;

  private static final HashMap<SuperState, LoggedTunableNumber> initializeSpeeds() {
    var map = new HashMap<SuperState, LoggedTunableNumber>();
    map.put(SuperState.STOW, new LoggedTunableNumber("Intake/StowSpeed", 0));
    map.put(SuperState.INTAKE, new LoggedTunableNumber("Intake/IntakeSpeed", -5));
    map.put(SuperState.L1, new LoggedTunableNumber("Intake/L1Speed", 2));
    map.put(SuperState.L2, new LoggedTunableNumber("Intake/L2Speed", 4));
    map.put(SuperState.L3, new LoggedTunableNumber("Intake/L3Speed", 4));
    map.put(SuperState.L4, new LoggedTunableNumber("Intake/L4Speed", 6));

    map.put(SuperState.L1PREPARE, map.get(SuperState.STOW));
    map.put(SuperState.L2PREPARE, map.get(SuperState.STOW));
    map.put(SuperState.L3PREPARE, map.get(SuperState.STOW));
    map.put(SuperState.L4PREPARE, map.get(SuperState.STOW));

    map.put(SuperState.INTAKEPREPARE, map.get(SuperState.STOW));
    map.put(SuperState.INTAKELOWPREPARE, map.get(SuperState.STOW));

    map.put(SuperState.L2L3ALGAE, map.get(SuperState.INTAKE));
    map.put(SuperState.L3L4ALGAE, map.get(SuperState.INTAKE));

    map.put(SuperState.INTAKELOW, map.get(SuperState.INTAKE));

    return map;
  }

  // private final LoggedTunableNumber intakeVelocity =
  //     new LoggedTunableNumber("Intake/IntakeVelocity", 2);
  // private final LoggedTunableNumber outtakeVelocity =
  //     new LoggedTunableNumber("Intake/OuttakeVelocity", -2);

  private @Getter @Setter SubState state = SubState.STOW;

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

    if (intakeSpeeds.containsKey(Superstructure.getCurrentState())) {
      intakeSpeed = intakeSpeeds.get(Superstructure.getCurrentState()).get();
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

  public BooleanSupplier intaked() {
    return () -> debounceTimer.get() > toleranceTime;
  }
}
