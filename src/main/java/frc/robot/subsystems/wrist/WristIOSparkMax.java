package frc.robot.subsystems.wrist;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class WristIOSparkMax implements WristIO {
  private static final double GEAR_RATIO = 45.0;

  private final SparkMax leader = new SparkMax(36, MotorType.kBrushless);
  private final SparkMaxConfig leaderConfig = new SparkMaxConfig();

  //   private final SparkMax follower = new SparkMax(45, MotorType.kBrushless);
  //   private final SparkMaxConfig followerconfig = new SparkMaxConfig();

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Wrist/Gains/kP", 6.0);
  //   private static final LoggedTunableNumber kI = new
  // LoggedTunableNumber("Wrist/Gains/kI", 0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Wrist/Gains/kD", 0.0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Wrist/Gains/kS", .5);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Wrist/Gains/kV", 12 / (5600.0 / 60.0) * (GEAR_RATIO)); // guess?
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Wrist/Gains/kA", 0);
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Wrist/Gains/kG", 0.7);

  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber(
          "Wrist/maxVelocity",
          // Units.rotationsPerMinuteToRadiansPerSecond((5600.0)) * (GEAR_RATIO) * .1
          150);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber(
          "Wrist/maxAcceleration",
          // Units.rotationsPerMinuteToRadiansPerSecond((5600.0)) * (GEAR_RATIO) * .1
          150);

  private final RelativeEncoder leader_encoder = leader.getEncoder();
  //   private final RelativeEncoder follower_encoder = follower.getEncoder();

  public static final Supplier<TrapezoidProfile.Constraints> maxProfileConstraints =
      () -> new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get());

  private ProfiledPIDController profile;
  private PIDController pid;
  private ArmFeedforward ff;
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

  double desiredPositionRads;

  public WristIOSparkMax() {

    profile =
        new ProfiledPIDController(
            kP.get(),
            0,
            kD.get(),
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()),
            0.02);

    // pid = new PIDController(kP.get() * 3, 0, kD.get());

    ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
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

    leader_encoder.setPosition(0);

    profile.reset(getPosition());

    // follower.configure(
    //     followerconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.connected = true;

    inputs.positionRad = getPosition();

    inputs.velocityRadPerSec = getSpeed();

    inputs.appliedVolts = ((leader.getAppliedOutput() * leader.getBusVoltage()));

    inputs.currentAmps = (leader.getOutputCurrent()) / 1.0;
  }

  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          profile.setPID(kP.get(), 0, kD.get());
        },
        kP,
        kD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            profile.setConstraints(
                new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get())),
        maxVelocity,
        maxAcceleration);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
        kS,
        kG,
        kV,
        kA);

    // profile.reset(profile.getSetpoint().position, profile.getSetpoint().velocity);

    // leader.setVoltage(pid.calculate(getPosition(), desiredPositionRads));
    leader.setVoltage(
        profile.calculate(getPosition(), profile.getSetpoint().position)
        // + ff.calculate(profile.getSetpoint().velocity)
        );

    Logger.recordOutput("Wrist/MaxVel", profile.getConstraints().maxVelocity);
    Logger.recordOutput("Wrist/MaxAccel", profile.getConstraints().maxAcceleration);

    Logger.recordOutput("Wrist/DistanceMeasured", desiredPositionRads - getPosition());

    Logger.recordOutput("Wrist/DistanceSetpoint", profile.getSetpoint().position);
    Logger.recordOutput("Wrist/desiredVelocity", profile.getSetpoint().velocity);

    // Logger.recordOutput("Wrist/ffVoltage", ff.calculate(profile.getSetpoint().velocity));

    // Logger.recordOutput(
    //     "Wrist/CalculatedVoltage",
    //     profile.calculate(getPosition(), profile.getSetpoint().position)
    //         + ff.calculate(profile.getSetpoint().velocity));

    Logger.recordOutput(
        "Wrist/PIDVelocityOutput", profile.calculate(getPosition(), desiredPositionRads));

    Logger.recordOutput("Wrist/desiredPositionRads", desiredPositionRads);
  }

  private double getPosition() {
    return Units.rotationsToRadians((leader_encoder.getPosition() / GEAR_RATIO));
  }

  private double getSpeed() {
    return Units.rotationsPerMinuteToRadiansPerSecond(leader_encoder.getVelocity() / GEAR_RATIO);
  }

  @Override
  public void stop() {
    leader.stopMotor();
    // follower.stopMotor();
  }

  @Override
  public void runPosition(double posRads) {

    // profile.reset(getPosition(), getSpeed());

    // constantly re-setting the goal might tweak the controller out
    if (desiredPositionRads != posRads) {

      desiredPositionRads = posRads;
      profile.reset(getPosition(), getSpeed());

      profile.setGoal(posRads);
    }
  }

  /** Resets the angle of the wrist to whatever we desire (rads) */
  @Override
  public void resetPosition(double posRads) {
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

  /**
   * Do be tuned, we increase the PID gains when we are nearby to reduce the error tolerance, it
   * tries harder to get closer
   */
  public void updateConstraints() {

    if (Math.abs(getPosition() - profile.getSetpoint().position) > .15) {
      profile.setPID(kP.get(), 0, kD.get());
    } else if (Math.abs(getPosition() - profile.getSetpoint().position) <= .15) {
      profile.setPID(kP.get() * 3, 0, kD.get() * 3);
    }
  }

  // @Override
  // public void configurePID(double kP, double kI, double kD) {
  //   pid.setP(kP);
  //   pid.setI(kI);
  //   pid.setD(kD);
  //   // pid.setFF(0);
  // }
}
