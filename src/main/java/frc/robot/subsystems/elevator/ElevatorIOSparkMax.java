package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.util.LoggedTunableNumber;

import java.security.Provider;
import java.util.function.Supplier;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class ElevatorIOSparkMax implements ElevatorIO {
  private static final double GEAR_RATIO = 10.0;
  public static final double maxElevatorRate = 5600.0 * GEAR_RATIO; // rpm

  private final SparkMax leader = new SparkMax(24, MotorType.kBrushless);
  private final SparkMaxConfig leaderConfig = new SparkMaxConfig();

//   private final SparkMax follower = new SparkMax(45, MotorType.kBrushless);
//   private final SparkMaxConfig followerconfig = new SparkMaxConfig();

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/Gains/kP", 0);
  //   private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/Gains/kI", 0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/Gains/kD", 0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/Gains/kS", 0);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "Arm/Gains/kV",
          12
              / ((5600.0 / 60.0)
                  / (GEAR_RATIO)
                  * 2
                  * Math.PI)); // it didn't want me to divide by 2PI on Kraken swerve, let's see I
  // guess?
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Arm/Gains/kA", 0);
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/Gains/kG", 0);

  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Arm/maxVelocity", .1);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Arm/maxAcceleration", .1);

  private final RelativeEncoder leader_encoder = leader.getEncoder();
//   private final RelativeEncoder follower_encoder = follower.getEncoder();

  public static final Supplier<TrapezoidProfile.Constraints> maxProfileConstraints =
      () -> new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get());

  private ProfiledPIDController profile;
  private ElevatorFeedforward ff;
//   private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();
  private TrapezoidProfile.State endState = new TrapezoidProfile.State();

//   private PIDController elevatorPID;

  ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
  ShuffleboardLayout intakeLayout =
      tab.getLayout("Intake", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);

  //   private final GenericEntry m_intakeRateEntry =
  //       intakeLayout.add("Intake Rate", 0 + " rpm").getEntry();
  //   private final GenericEntry m_rotateAngleEntry =
  //       intakeLayout.add("Intake Angle", 0 + " rad").getEntry();
  //   private final GenericEntry m_rotateAngularSpeedEntry =
  //       intakeLayout.add("Intake Angular Speed", 0 + " rad/s").getEntry();

  public ElevatorIOSparkMax() {

    profile = new ProfiledPIDController(kP.get(), 0, kD.get(),
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));


    
    ff = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
    // elevatorPID = new PIDController(kP.get(), 0, kD.get());

    leaderConfig.inverted(false);

    // followerconfig.inverted(true); //inverting already done in the next line
    // followerconfig.follow(leader.getDeviceId(), true);

    leaderConfig.idleMode(IdleMode.kBrake);
    // followerconfig.idleMode(IdleMode.kBrake);

    // leaderConfig.signals.absoluteEncoderPositionPeriodMs(20);
    // followerconfig.signals.absoluteEncoderPositionPeriodMs(20);

    // leaderConfig.signals.absoluteEncoderVelocityPeriodMs(20);
    // followerconfig.signals.absoluteEncoderVelocityPeriodMs(20);

    leaderConfig.smartCurrentLimit(80, 60);
    // followerconfig.smartCurrentLimit(80, 60);

    leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // follower.configure(
    //     followerconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.motorType = "Sparkmax";

    inputs.positionRads = getHeight();

    inputs.velocityRadsPerSec =
        Units.rotationsToRadians(
            leader_encoder.getVelocity() / GEAR_RATIO);

    inputs.appliedVoltage =
        ((leader.getAppliedOutput() * leader.getBusVoltage()));

    inputs.supplyCurrentAmps = (leader.getOutputCurrent()) / 1.0;

    inputs.torqueCurrentAmps = 0; // can't measure torque/stator current output for sparkamx

    inputs.avgTempCelsius = (leader.getMotorTemperature()) / 1.0;
  }

  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            profile.setConstraints(
                    new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get())),
        maxVelocity,
        maxAcceleration);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> ff = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
        kS,
        kG,
        kV,
        kA);


    profile.reset(profile.getSetpoint().position, profile.getSetpoint().velocity);

    leader.setVoltage(
        profile.calculate(getHeight(), profile.getSetpoint().position)
            + ff.calculate(profile.getSetpoint().velocity));
  }

  private double getHeight() {
    return Units.rotationsToRadians(
        (leader_encoder.getPosition() / GEAR_RATIO));
  }

  private double getElevatorSpeed(){
    return Units.rotationsPerMinuteToRadiansPerSecond(
        leader_encoder.getVelocity() / GEAR_RATIO
    );
  }

  @Override
  public void stop() {
    leader.stopMotor();
    // follower.stopMotor();
  }

  @Override
  public void movetoHeight(double posRads) {
    profile.reset(
        getHeight(), getElevatorSpeed());
    
    profile.setGoal(new TrapezoidProfile.State(posRads, 0));
  }

  /** Resets the angle of the elevator to whatever we desire (rads) */
  @Override
  public void setHeight(double posRads) {
    leader_encoder.setPosition(Units.radiansToRotations(posRads));
  }

  @Override
  public void setBrakeMode(boolean enable) {
    leaderConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    // followerConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // follower.configure(
    //     followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Displays the periodically updated intake rate on the Shuffleboard */
  public void updateShuffleboard() {
    //   m_intakeRateEntry.setString(intake_encoder.getVelocity() + " rpm");
    //   m_rotateAngleEntry.setString(rotate_encoder.getPosition() + " rad");
    //   m_rotateAngularSpeedEntry.setString(rotate_encoder.getVelocity() + " rad/s");

  }

  // @Override
  // public void configurePID(double kP, double kI, double kD) {
  //   pid.setP(kP);
  //   pid.setI(kI);
  //   pid.setD(kD);
  //   // pid.setFF(0);
  // }
}
