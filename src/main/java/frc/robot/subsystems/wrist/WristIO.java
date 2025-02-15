package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void runPosition(double position) {}

  public default void runVolts(double volts) {}

  public default void stop() {}

  public default void periodic() {}

  public default void setBrakeMode(boolean enable) {}

  public default void resetPosition(double position) {}
}
