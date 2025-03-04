package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableNumber;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class IntakeIOTalonFX implements IntakeIO {
  final VoltageOut VoltageRequest = new VoltageOut(0);
  // final VelocityTorqueCurrentFOC torquerequest = new VelocityTorqueCurrentFOC(0);

  private final TalonFX talon;
  private static TalonFXConfiguration talonConfig = new TalonFXConfiguration();

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/Gains/kP", 100);
  // private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/Gains/kI", 0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/Gains/kD", 0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/Gains/kS", 0);
  // kV is Voltage given per unit of velocity, in this case volts / rad / s
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Intake/Gains/kV", 12.0 / 5600.0);
  // kA is Voltage given per unit of acceleration, volts / rad / s^2
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake/Gains/kA", 0);

  private static final LoggedTunableNumber motionMagicAcceleration =
      new LoggedTunableNumber("Intake/maxAcceleration", .1);
  private static final LoggedTunableNumber motionMagicJerk =
      new LoggedTunableNumber("Intake/maxJerk", .1);

  private final StatusSignal<Angle> intakePosition;
  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Voltage> intakeAppliedVolts;
  private final StatusSignal<Current> intakeCurrent;
  private final StatusSignal<Current> intakeTorqueCurrent;

  private final Debouncer intakeConnectedDebounce = new Debouncer(0.5);

  //   ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
  //   ShuffleboardLayout intakeLayout = tab.getLayout("Intake", BuiltInLayouts.kList).withSize(2,
  // 4).withPosition(0, 0);
  //   private final GenericEntry m_intakeRateEntry = intakeLayout.add("Intake Rate", 0 + "
  // rpm").getEntry();
  //   private final GenericEntry m_rotateAngleEntry = intakeLayout.add("Intake Angle", 0 + "
  // rad").getEntry();
  //   private final GenericEntry m_rotateAngularSpeedEntry = intakeLayout.add("Intake Angular
  // Speed", 0 + " rad/s").getEntry();

  public IntakeIOTalonFX() {
    talon = new TalonFX(23, "rio");

    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // talonConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    talonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    talonConfig.Slot0.kA = kA.get();
    talonConfig.Slot0.kD = kD.get();
    // talonConfig.Slot0.kG = kG.get();
    talonConfig.Slot0.kP = kP.get();
    talonConfig.Slot0.kS = kS.get();
    talonConfig.Slot0.kV = kV.get();

    talonConfig.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration.get();
    // talonConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicVelocity.get();
    talonConfig.MotionMagic.MotionMagicJerk = motionMagicJerk.get();

    // talonConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    // talonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    talonConfig.CurrentLimits.StatorCurrentLimit = 60;
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonConfig.CurrentLimits.SupplyCurrentLimit = 50;
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // false // fix this, test this.  Positive should be upward
    //     ? InvertedValue.Clockwise_Positive
    //     : InvertedValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
    tryUntilOk(5, () -> talon.setPosition(0.0, 0.25));

    intakePosition = talon.getPosition();
    intakeVelocity = talon.getVelocity();
    intakeAppliedVolts = talon.getMotorVoltage();
    intakeCurrent = talon.getSupplyCurrent();
    intakeTorqueCurrent = talon.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        intakePosition,
        intakeVelocity,
        intakeAppliedVolts,
        intakeCurrent,
        intakeTorqueCurrent);
    ParentDevice.optimizeBusUtilizationForAll(talon);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          talonConfig.Slot0.kA = kA.get();
          talonConfig.Slot0.kD = kD.get();
          // talonConfig.Slot0.kG = kG.get();
          talonConfig.Slot0.kP = kP.get();
          talonConfig.Slot0.kS = kS.get();
          talonConfig.Slot0.kV = kV.get();
          tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
        },
        kA,
        kD,
        // kG,
        kP,
        kS,
        kV);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          talonConfig.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration.get();
          // talonConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicVelocity.get();
          talonConfig.MotionMagic.MotionMagicJerk = motionMagicJerk.get();
          tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
        },
        motionMagicAcceleration,
        motionMagicJerk);
    var talonStatus =
        BaseStatusSignal.refreshAll(
            intakePosition, intakeVelocity, intakeAppliedVolts, intakeCurrent, intakeTorqueCurrent);

    inputs.connected = intakeConnectedDebounce.calculate(talonStatus.isOK());

    inputs.positionRad = Units.rotationsToRadians(intakePosition.getValueAsDouble());
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(intakeVelocity.getValueAsDouble());
    inputs.appliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.currentAmps = intakeCurrent.getValueAsDouble();
    // inputs.torqueCurrentAmps = intakeTorqueCurrent.getValueAsDouble();
  }

  // @Override
  // public void stop() {
  //   talon.stopMotor();
  // }

  /** Resets the angle of the intake to 0. */
  public void set(double positionRads) {
    talon.setPosition(Units.radiansToRotations(positionRads));
  }

  /** Run intake with velocity */
  public void setVelocity(double velocityRadPerSec) {
    talon.setControl(VoltageRequest.withOutput((velocityRadPerSec)));
    // talon.setControl(torquerequest.withVelocity(velocityRadPerSec));
  }

  //   /** Displays the periodically updated intake rate on the Shuffleboard */
  //   public void updateShuffleboard() {
  //       m_intakeRateEntry.setString(intake_encoder.getVelocity() + " rpm");
  //       m_rotateAngleEntry.setString(rotate_encoder.getPosition() + " rad");
  //       m_rotateAngularSpeedEntry.setString(rotate_encoder.getVelocity() + " rad/s");

  //   }

  // @Override
  // public void configurePID(double kP, double kI, double kD) {
  //   pid.setP(kP);
  //   pid.setI(kI);
  //   pid.setD(kD);
  //   // pid.setFF(0);
  // }

}
