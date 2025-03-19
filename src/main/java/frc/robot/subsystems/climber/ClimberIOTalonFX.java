package frc.robot.subsystems.climber;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class ClimberIOTalonFX implements ClimberIO {

  private final TalonFX talon;

  Debouncer climberConnectedDebounce = new Debouncer(.5);

  // private final TalonFX followertalon;
  private static TalonFXConfiguration talonConfig = new TalonFXConfiguration();

  private final StatusSignal<Angle> climberPosition;
  private final StatusSignal<AngularVelocity> climberVelocity;
  private final StatusSignal<Voltage> climberAppliedVolts;
  private final StatusSignal<Current> climberCurrent;
  private final StatusSignal<Current> climberTorqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  public ClimberIOTalonFX() {
    talon = new TalonFX(18, "rio");

    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    talonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    talonConfig.CurrentLimits.StatorCurrentLimit = 120;
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonConfig.CurrentLimits.SupplyCurrentLimit = 80;
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    talonConfig.MotorOutput.Inverted =
        false // fix this, test this.  Positive should be upward
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
    tryUntilOk(5, () -> talon.setPosition(0.0, 0.25));

    climberPosition = talon.getPosition();
    climberVelocity = talon.getVelocity();
    climberAppliedVolts = talon.getMotorVoltage();
    climberCurrent = talon.getSupplyCurrent();
    climberTorqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        climberPosition,
        climberVelocity,
        climberAppliedVolts,
        climberCurrent,
        climberTorqueCurrent,
        tempCelsius);
    ParentDevice.optimizeBusUtilizationForAll(talon);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    var talonStatus =
        BaseStatusSignal.refreshAll(
            climberPosition,
            climberVelocity,
            climberAppliedVolts,
            climberCurrent,
            climberTorqueCurrent,
            tempCelsius);

    inputs.connected = climberConnectedDebounce.calculate(talonStatus.isOK());

    inputs.positionRad = Units.rotationsToRadians(climberPosition.getValueAsDouble());
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(climberVelocity.getValueAsDouble());
    inputs.appliedVolts = climberAppliedVolts.getValueAsDouble();
    inputs.currentAmps = climberTorqueCurrent.getValueAsDouble();
  }

  public void stop() {
    talon.stopMotor();
  }

  /** Resets the angle of the intake to 0. */
  public void resetPosition(double positionRads) {
    talon.setPosition(Units.radiansToRotations(positionRads));
  }

  /** Run climber with voltage */
  public void setvoltage(double voltage) {
    talon.setControl(new VoltageOut(voltage));
  }
}
