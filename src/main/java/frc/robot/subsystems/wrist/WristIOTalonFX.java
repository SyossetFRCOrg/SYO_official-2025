package frc.robot.subsystems.wrist;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import frc.robot.util.LoggedTunableNumber;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class WristIOTalonFX implements WristIO {

  private static final double GEAR_RATIO = 25.0;
  // public static final double maxspeed = 5600.0 / GEAR_RATIO; // rpm

  private final TalonFX talon;
  // private final TalonFX followertalon;
  private static TalonFXConfiguration talonConfig = new TalonFXConfiguration();

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Wrist/Gains/kP", 300);
  // private static final LoggedTunableNumber kI = new LoggedTunableNumber("Wrist/Gains/kI", 0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Wrist/Gains/kD", 0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Wrist/Gains/kS", 0);
  // kV is Voltage given per unit of velocity, in this case volts / rad / s
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Wrist/Gains/kV", 0);
  // kA is Voltage given per unit of acceleration, volts / rad / s^2
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Wrist/Gains/kA", 0);
  // kG is a constant voltage needed to keep the wrist at that height, the Voltage needed to
  // counteract gravity
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Wrist/Gains/kG", 0);

  private static final LoggedTunableNumber motionMagicVelocity =
      new LoggedTunableNumber("Wrist/maxVelocity", 4);
  private static final LoggedTunableNumber motionMagicAcceleration =
      new LoggedTunableNumber("Wrist/maxAcceleration", 4);
  private static final LoggedTunableNumber motionMagicJerk =
      new LoggedTunableNumber("Wrist/maxJerk", 1000);

  private final StatusSignal<Angle> wristPosition;
  private final StatusSignal<AngularVelocity> wristVelocity;
  private final StatusSignal<Voltage> wristAppliedVolts;
  private final StatusSignal<Current> wristCurrent;
  private final StatusSignal<Current> wristTorqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  final MotionMagicVoltage wristRequest = new MotionMagicVoltage(0);

  private final Debouncer wristConnectedDebounce = new Debouncer(0.5);

  //   ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
  //   ShuffleboardLayout intakeLayout = tab.getLayout("Intake", BuiltInLayouts.kList).withSize(2,
  // 4).withPosition(0, 0);
  //   private final GenericEntry m_intakeRateEntry = intakeLayout.add("Intake Rate", 0 + "
  // rpm").getEntry();
  //   private final GenericEntry m_rotateAngleEntry = intakeLayout.add("Intake Angle", 0 + "
  // rad").getEntry();
  //   private final GenericEntry m_rotateAngularSpeedEntry = intakeLayout.add("Intake Angular
  // Speed", 0 + " rad/s").getEntry();

  public WristIOTalonFX() {
    talon = new TalonFX(22, "rio");
    // followertalon = new TalonFX(17, "rio");

    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    talonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    talonConfig.Slot0.kA = kA.get();
    talonConfig.Slot0.kD = kD.get();
    talonConfig.Slot0.kG = kG.get();
    talonConfig.Slot0.kP = kP.get();
    talonConfig.Slot0.kS = kS.get();
    talonConfig.Slot0.kV = kV.get();

    talonConfig.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration.get();
    // talonConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicVelocity.get();
    talonConfig.MotionMagic.MotionMagicCruiseVelocity = 1;

    talonConfig.MotionMagic.MotionMagicJerk = motionMagicJerk.get();

    talonConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    talonConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60;
    talonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -60;
    talonConfig.CurrentLimits.StatorCurrentLimit = 80;
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonConfig.CurrentLimits.SupplyCurrentLimit = 60;
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // tryUntilOk(5, () -> followertalon.getConfigurator().apply(talonConfig, 0.25));

    talonConfig.MotorOutput.Inverted =
        false // fix this, test this.  Positive should be upward
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
    tryUntilOk(5, () -> talon.setPosition(0.0, 0.25));

    wristPosition = talon.getPosition();
    wristVelocity = talon.getVelocity();
    wristAppliedVolts = talon.getMotorVoltage();
    wristCurrent = talon.getSupplyCurrent();
    wristTorqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        wristPosition,
        wristVelocity,
        wristAppliedVolts,
        wristCurrent,
        wristTorqueCurrent,
        tempCelsius);
    ParentDevice.optimizeBusUtilizationForAll(talon);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          talonConfig.Slot0.kA = kA.get();
          talonConfig.Slot0.kD = kD.get();
          talonConfig.Slot0.kG = kG.get();
          talonConfig.Slot0.kP = kP.get();
          talonConfig.Slot0.kS = kS.get();
          talonConfig.Slot0.kV = kV.get();
          tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
        },
        kA,
        kD,
        kG,
        kP,
        kS,
        kV);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          talonConfig.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration.get();
          talonConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicVelocity.get();
          talonConfig.MotionMagic.MotionMagicJerk = motionMagicJerk.get();
          tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
          // wristRequest.Velocity =  motionMagicVelocity.get();
          // wristRequest.Acceleration =  motionMagicAcceleration.get();
          // wristRequest.Jerk =  motionMagicJerk.get();
        },
        motionMagicAcceleration,
        motionMagicJerk,
        motionMagicVelocity);
    var talonStatus =
        BaseStatusSignal.refreshAll(
            wristPosition,
            wristVelocity,
            wristAppliedVolts,
            wristCurrent,
            wristTorqueCurrent,
            tempCelsius);

    inputs.connected = wristConnectedDebounce.calculate(talonStatus.isOK());

    inputs.positionRad = Units.rotationsToRadians(wristPosition.getValueAsDouble());
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(wristVelocity.getValueAsDouble());
    inputs.appliedVolts = wristAppliedVolts.getValueAsDouble();
    inputs.currentAmps = wristTorqueCurrent.getValueAsDouble();
  }

  @Override
  public void stop() {
    talon.stopMotor();
  }

  /** Resets the angle of the intake to 0. */
  public void resetPosition(double positionRads) {
    talon.setPosition(Units.radiansToRotations(positionRads));
  }

  /** Run wrist to position - Motion Magic */
  public void runPosition(double posRads) {
    talon.setControl(wristRequest.withPosition(Units.radiansToRotations(posRads)));
    // talon.setControl(new VoltageOut(-1));

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
