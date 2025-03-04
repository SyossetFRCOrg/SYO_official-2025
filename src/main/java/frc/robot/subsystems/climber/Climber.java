package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private double voltage = 0;

  private final ClimberIO climberIO;

  public Climber(ClimberIO climberIO) {
    this.climberIO = climberIO;

    // new ControllerRumbleCommand(new XboxController(0), () ->
    // debouncer.calculate(inputs.currentAmps > 30));

  }

  @Override
  public void periodic() {
    climberIO.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    climberIO.setvoltage(voltage);
  }

  public Command setMotorVoltage(double voltage) {
    return new InstantCommand(() -> this.voltage = voltage);
  }

  //   public BooleanSupplier climbed() {
  //     return () -> debounceTimer.get() > toleranceTime;
  //   }
}
