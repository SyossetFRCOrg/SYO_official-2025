package frc.robot.subsystems.wrist;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class WristIOSparkMax implements WristIO {
  private final SparkMax sparkMax;
  private final RelativeEncoder encoder;
  private final SparkMaxConfig sparkConfig = new SparkMaxConfig();

  private final double GEAR_RATIO = 1;

  private final Debouncer connectedDebounce = new Debouncer(0.5);

  private SimpleMotorFeedforward ff;
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Wrist/kP", 0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Wrist/kD", 0);
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Wrist/MaxVelocity", 0.3);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Wrist/MaxAcceleration", 0.1);

  private ProfiledPIDController pidController =
      new ProfiledPIDController(
          kP.get(),
          0,
          kD.get(),
          new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));

  public WristIOSparkMax() {
    ff = new SimpleMotorFeedforward(0, 12 / 5600 * GEAR_RATIO, 0);

    
    sparkMax = new SparkMax(-2, MotorType.kBrushless);
    encoder = sparkMax.getEncoder();

    sparkConfig.inverted(false);
    sparkConfig.idleMode(IdleMode.kBrake);

    sparkMax.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pidController.reset(Units.rotationsToRadians(encoder.getPosition()));

  }

  @Override
  public void runPosition(double position) {
    pidController.setGoal(position);

    sparkMax.setVoltage(
        ff.calculate(pidController.getSetpoint().velocity) +
        pidController.calculate(Units.radiansToRotations(position), encoder.getPosition()));
  }

  @Override
  public void resetPosition(double position) {
    encoder.setPosition(Units.radiansToRotations(position));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
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
}
