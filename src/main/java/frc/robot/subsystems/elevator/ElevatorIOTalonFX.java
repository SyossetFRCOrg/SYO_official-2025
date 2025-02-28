package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
public class ElevatorIOTalonFX implements ElevatorIO {

  private static final double GEAR_RATIO = 4.6875 * 5.0 / 3.0;
  // public static final double maxspeed = 5600.0 / GEAR_RATIO; // rpm

  

  private final TalonFX talon;
  // private final TalonFX followertalon;
  private static TalonFXConfiguration talonConfig = new TalonFXConfiguration();

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/Gains/kP", 1000);
  // private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/Gains/kI", 0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/Gains/kD", 20);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/Gains/kS", 0);
  // kV is Voltage given per unit of velocity, in this case volts / rad / s
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Arm/Gains/kV", 0);
  // kA is Voltage given per unit of acceleration, volts / rad / s^2
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Arm/Gains/kA", 0);
  // kG is a constant voltage needed to keep the elevator at that height, the Voltage needed to
  // counteract gravity
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/Gains/kG", 0);

  private static final LoggedTunableNumber motionMagicVelocity =
      new LoggedTunableNumber("Arm/maxVelocity", .2);
  private static final LoggedTunableNumber motionMagicAcceleration =
      new LoggedTunableNumber("Arm/maxAcceleration", .1);
  private static final LoggedTunableNumber motionMagicJerk =
      new LoggedTunableNumber("Arm/maxJerk", 100);

  private final StatusSignal<Angle> elevatorPosition;
  private final StatusSignal<AngularVelocity> elevatorVelocity;
  private final StatusSignal<Voltage> elevatorAppliedVolts;
  private final StatusSignal<Current> elevatorCurrent;
  private final StatusSignal<Current> elevatorTorqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  final DynamicMotionMagicTorqueCurrentFOC elevatorRequest = new DynamicMotionMagicTorqueCurrentFOC(0, motionMagicVelocity.get(), motionMagicAcceleration.get(), motionMagicJerk.get());

  private final Debouncer elevatorConnectedDebounce = new Debouncer(0.5);

  //   ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
  //   ShuffleboardLayout intakeLayout = tab.getLayout("Intake", BuiltInLayouts.kList).withSize(2,
  // 4).withPosition(0, 0);
  //   private final GenericEntry m_intakeRateEntry = intakeLayout.add("Intake Rate", 0 + "
  // rpm").getEntry();
  //   private final GenericEntry m_rotateAngleEntry = intakeLayout.add("Intake Angle", 0 + "
  // rad").getEntry();
  //   private final GenericEntry m_rotateAngularSpeedEntry = intakeLayout.add("Intake Angular
  // Speed", 0 + " rad/s").getEntry();

  public ElevatorIOTalonFX() {
    talon = new TalonFX(16, "rio");
    // followertalon = new TalonFX(17, "rio");

    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    talonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    talonConfig.Slot0.kA = kA.get();
    talonConfig.Slot0.kD = kD.get();
    talonConfig.Slot0.kG = kG.get();
    talonConfig.Slot0.kP = kP.get();
    talonConfig.Slot0.kS = kS.get();
    talonConfig.Slot0.kV = kV.get();

    talonConfig.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration.get();
    talonConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicVelocity.get();
    talonConfig.MotionMagic.MotionMagicJerk = motionMagicJerk.get();

    talonConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    talonConfig.TorqueCurrent.PeakForwardTorqueCurrent = 120;
    talonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -120;
    talonConfig.CurrentLimits.StatorCurrentLimit = 120;
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonConfig.CurrentLimits.SupplyCurrentLimit = 80;
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // tryUntilOk(5, () -> followertalon.getConfigurator().apply(talonConfig, 0.25));

    talonConfig.MotorOutput.Inverted =
        false // fix this, test this.  Positive should be upward
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
    tryUntilOk(5, () -> talon.setPosition(0.0, 0.25));

    elevatorPosition = talon.getPosition();
    elevatorVelocity = talon.getVelocity();
    elevatorAppliedVolts = talon.getMotorVoltage();
    elevatorCurrent = talon.getSupplyCurrent();
    elevatorTorqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        elevatorPosition,
        elevatorVelocity,
        elevatorAppliedVolts,
        elevatorCurrent,
        elevatorTorqueCurrent,
        tempCelsius);
    ParentDevice.optimizeBusUtilizationForAll(talon);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
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
          elevatorRequest.Velocity =  motionMagicVelocity.get();
          elevatorRequest.Acceleration =  motionMagicAcceleration.get();
          elevatorRequest.Jerk =  motionMagicJerk.get();          
        },
        motionMagicAcceleration,
        motionMagicJerk,
        motionMagicVelocity);
    var talonStatus =
        BaseStatusSignal.refreshAll(
            elevatorPosition,
            elevatorVelocity,
            elevatorAppliedVolts,
            elevatorCurrent,
            elevatorTorqueCurrent,
            tempCelsius);

    inputs.motorType = "Kraken";

    inputs.motorConnected = elevatorConnectedDebounce.calculate(talonStatus.isOK());

    inputs.positionRads = Units.rotationsToRadians(elevatorPosition.getValueAsDouble());
    inputs.velocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(elevatorVelocity.getValueAsDouble());
    inputs.appliedVoltage = elevatorAppliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = elevatorCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = elevatorTorqueCurrent.getValueAsDouble();
    inputs.avgTempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void stop() {
    talon.stopMotor();
  }

  /** Resets the angle of the intake to 0. */
  public void set(double positionRads) {
    talon.setPosition(Units.radiansToRotations(positionRads));
  }

  /** Run elevator to position - Motion Magic */
  public void movetoHeight(double posRads) {
    if (posRads > Units.rotationsToRadians(elevatorPosition.getValueAsDouble())) {
      elevatorRequest.Velocity = motionMagicVelocity.get();
      elevatorRequest.Acceleration = motionMagicAcceleration.get();
      elevatorRequest.Jerk = motionMagicJerk.get();

    } else if (posRads < Units.rotationsToRadians(elevatorPosition.getValueAsDouble())) {
      elevatorRequest.Velocity = motionMagicVelocity.get() / 3.0;
      elevatorRequest.Acceleration = motionMagicAcceleration.get() / 3.0;
      elevatorRequest.Jerk = motionMagicJerk.get() / 3.0;
    }
    talon.setControl(elevatorRequest.withPosition(Units.radiansToRotations(posRads)));
    // followertalon.setControl(new Follower(talon.getDeviceID(), true));
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
