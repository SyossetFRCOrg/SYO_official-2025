package frc.robot.subsystems.climber;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;

public class ClimberIOSparkMax implements ClimberIO {
  private final SparkMax sparkMax;
  private final RelativeEncoder encoder;
  private final SparkMaxConfig sparkConfig = new SparkMaxConfig();

  private final Debouncer connectedDebounce = new Debouncer(0.5);

  // private final SimpleMotorFeedforward ff =
  //     new SimpleMotorFeedforward(0, 12 / Units.rotationsPerMinuteToRadiansPerSecond(5600));

  public ClimberIOSparkMax() {
    sparkMax = new SparkMax(24, MotorType.kBrushless);
    encoder = sparkMax.getEncoder();

    sparkConfig.inverted(false);
    sparkConfig.idleMode(IdleMode.kBrake);
    sparkConfig.smartCurrentLimit(80);

    sparkMax.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    sparkStickyFault = false;

    ifOk(
        sparkMax,
        encoder::getPosition,
        (value) -> inputs.positionRad = Units.rotationsToRadians(value));
    ifOk(
        sparkMax,
        encoder::getVelocity,
        (value) -> inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(value));
    ifOk(
        sparkMax,
        new DoubleSupplier[] {sparkMax::getAppliedOutput, sparkMax::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);

    ifOk(sparkMax, sparkMax::getOutputCurrent, (value) -> inputs.currentAmps = value);

    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setvoltage(double voltage) {
    if (voltage != 0) {
      sparkMax.setVoltage(voltage);
    } else {
      sparkMax.stopMotor();
    }
  }
}
