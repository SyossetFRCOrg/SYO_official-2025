package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;

public class Intake extends SubsystemBase {
  public static enum SubState {
    INTAKE_IN,
    INTAKE_OUT,
    INTAKING,
    OUTTAKING,
    STOW,

  }

  private final LoggedTunableNumber intakeVelocity =
      new LoggedTunableNumber("Intake/IntakeVelocity", 1);
  private final LoggedTunableNumber outtakeVelocity =
      new LoggedTunableNumber("Intake/OuttakeVelocity", -1);

  private @Getter @Setter SubState state = SubState.INTAKE_OUT;

  private final IntakeIO intakeIO;

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    switch (state) {
      case INTAKING:
        intakeIO.setVelocity(intakeVelocity.get());
        break;
      case OUTTAKING:
        intakeIO.setVelocity(outtakeVelocity.get());
        break;
      case STOW:
      default:
        intakeIO.setVelocity(0);
        break;
    }
  }
}
