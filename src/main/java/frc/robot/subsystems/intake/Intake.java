package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.util.LoggedTunableNumber;
import java.util.HashMap;
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

  private static final HashMap<SuperState, LoggedTunableNumber> intakeSpeeds =
      initializePositions();

  private static final HashMap<SuperState, LoggedTunableNumber> initializePositions() {
    var map = new HashMap<SuperState, LoggedTunableNumber>();
    map.put(SuperState.STOW, new LoggedTunableNumber("Intake/StowPosition", 0));
    map.put(SuperState.INTAKE, new LoggedTunableNumber("Intake/IntakePosition", -3));
    map.put(SuperState.L1, new LoggedTunableNumber("Intake/L1Position", 4));
    map.put(SuperState.L2, new LoggedTunableNumber("Intake/L2Position", 4));
    map.put(SuperState.L3, new LoggedTunableNumber("Intake/L3Position", 4));
    map.put(SuperState.L4, new LoggedTunableNumber("Intake/L4Position", 4));

    map.put(SuperState.L1PREPARE, map.get(SuperState.STOW));
    map.put(SuperState.L2PREPARE, map.get(SuperState.STOW));
    map.put(SuperState.L3PREPARE, map.get(SuperState.STOW));
    map.put(SuperState.L4PREPARE, map.get(SuperState.STOW));

    map.put(SuperState.INTAKEPREPARE, map.get(SuperState.STOW));

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
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (intakeSpeeds.containsKey(Superstructure.getCurrentState())) {
      intakeSpeed = intakeSpeeds.get(Superstructure.getCurrentState()).get();
    }

    intakeIO.setVelocity(intakeSpeed); // intakeSpeed * 6
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

  public void setIntakeState(SubState state) {
    this.state = state;
  }
}
