package frc.robot.util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import java.util.function.Supplier;

/**
 * The fact that users have to make classes just to be able to use motor controllers you are already
 * paying $90 or even $200 for is UNACCEPTABLE. This is not okay REV, I should not have to use this
 * to make your product usable on the software end.
 */
public class SparkMaxUtil {
  public static final int disableFramePeriod = 32767;

  public static void configureFollower(CANSparkMax follower) {
    configureNoReset(
        follower,
        () -> follower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, disableFramePeriod),
        () -> follower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, disableFramePeriod),
        () -> follower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, disableFramePeriod),
        () -> follower.setPeriodicFramePeriod(PeriodicFrame.kStatus3, disableFramePeriod),
        () -> follower.setPeriodicFramePeriod(PeriodicFrame.kStatus4, disableFramePeriod),
        () -> follower.setPeriodicFramePeriod(PeriodicFrame.kStatus5, disableFramePeriod),
        () -> follower.setPeriodicFramePeriod(PeriodicFrame.kStatus6, disableFramePeriod),
        () -> follower.setPeriodicFramePeriod(PeriodicFrame.kStatus7, disableFramePeriod));
  }

  public static final int MAX_ATTEMPTS = 5;

  /**
   * Formats the name of a spark with its CAN ID.
   *
   * @param spark The spark to find the name of.
   * @return The name of a spark.
   */
  public static String name(CANSparkBase spark) {
    return "Spark Max [" + spark.getDeviceId() + "]";
  }

  /**
   * This is a workaround since {@link CANSparkBase#setInverted(boolean)} does not return a {@code
   * REVLibError} because it is overriding {@link
   * edu.wpi.first.wpilibj.motorcontrol.MotorController}.
   *
   * <p>This call has no effect if the controller is a follower. To invert a follower, see the
   * follow() method.
   *
   * @param spark The spark to set inversion of.
   * @param isInverted The state of inversion, true is inverted.
   * @return {@link REVLibError#kOk} if successful.
   */
  public static REVLibError setInverted(CANSparkBase spark, boolean isInverted) {
    spark.setInverted(isInverted);
    return spark.getLastError();
  }

  public static REVLibError setPeriodicTimeout(CANSparkBase spark, int timeout) {
    spark.setPeriodicFrameTimeout(timeout);
    return spark.getLastError();
  }

  /**
   * Fully configures a Spark Max/Flex with all provided configs.
   *
   * <p>Each config is applied until success, or until the number of attempts exceed {@code
   * MAX_ATTEMPTS}.
   *
   * @param spark The spark to configure.
   * @param config The configuration to apply.
   */
  @SafeVarargs
  public static void configure(CANSparkBase spark, Supplier<REVLibError>... config) {
    spark.setCANMaxRetries(MAX_ATTEMPTS);
    configure(spark, spark::restoreFactoryDefaults);
    configure(spark, () -> spark.setCANTimeout(100));
    for (var f : config) {
      configure(spark, f::get);
    }
    configure(spark, () -> spark.setCANTimeout(0));
    configure(spark, () -> setPeriodicTimeout(spark, 0));
  }

  @SafeVarargs
  public static void configureNoReset(CANSparkBase spark, Supplier<REVLibError>... config) {
    spark.setCANMaxRetries(MAX_ATTEMPTS);
    configure(spark, () -> spark.setCANTimeout(100));
    for (var f : config) {
      configure(spark, f::get);
    }
    configure(spark, () -> spark.setCANTimeout(0));
    configure(spark, () -> setPeriodicTimeout(spark, 0));
  }

  /**
   * Recursively configures a specific value on a spark, until {@code attempt} exceeds {@code
   * MAX_ATTEMPTS}.
   *
   * @param spark The spark to configure.
   * @param config The configuration to apply to the spark.
   * @param attempt The current attempt number.
   */
  private static void configure(CANSparkBase spark, Supplier<REVLibError> config) {
    for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
      REVLibError error = config.get();
      if (error != REVLibError.kOk) {
        if (attempt >= MAX_ATTEMPTS) {
          FaultLogger.error(name(spark), "Failed to set parameter!");
        } else {
          FaultLogger.warning(
              name(spark), "Failed to set parameter: attempt " + attempt + "/" + MAX_ATTEMPTS);
        }
        continue;
      } else {
        break;
      }
    }
  }
}