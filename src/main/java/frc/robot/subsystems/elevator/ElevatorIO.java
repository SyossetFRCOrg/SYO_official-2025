package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public String motorType = "";
    public boolean motorConnected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double avgTempCelsius = 0.0;
  }

  /** Update the inputs. */
  default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run elevator to position - Motion Magic */
  default void movetoHeight(double posRads) {}

  /** Sets the elevator to a height, as in "resetting" the elevator */
  default void setHeight(double posRads) {}

  /**
   * used only for SparkMaxIO, in order to properly run the trapezoidal profile. Not needed for
   * motion magic
   */
  default void periodic() {}

  /** Stop slam elevator */
  default void stop() {}

  /** Enable or disable brake mode on the elevator motor. */
  default void setBrakeMode(boolean enable) {}

  /** Displays the periodically updated outtake rate on the Shuffleboard */
  public default void updateShuffleboard() {}
}
