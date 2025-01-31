package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class IntakeIOSparkMax implements IntakeIO {
  private final SparkMax sparkMax;
  private final RelativeEncoder encoder;
  private final SparkMaxConfig sparkConfig = new SparkMaxConfig();

  public IntakeIOSparkMax() {
    sparkMax = new SparkMax(-2, MotorType.kBrushless);
    encoder = sparkMax.getEncoder();

    sparkConfig.inverted(false);
    sparkConfig.idleMode(IdleMode.kCoast);

    sparkMax.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition());
    inputs.velocityRadPerSec = Units.rotationsToRadians(encoder.getVelocity());
    inputs.currentAmps = sparkMax.getOutputCurrent();
    inputs.appliedVolts = sparkMax.getAppliedOutput() * sparkMax.getBusVoltage();
  }

  @Override
  public void setVelocity(double velocity) {
    sparkMax.set(Units.radiansToRotations(velocity));
  }
}
