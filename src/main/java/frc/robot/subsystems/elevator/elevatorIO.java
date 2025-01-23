// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface elevatorIO {
  @AutoLog
  class elevatorIOInputs {
    public boolean motorConnected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double avgTempCelsius = 0.0;
    // public TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0,0));
    // public TrapezoidProfile.State trapezoidalProfileState = new TrapezoidProfile.State();
  }

  /** Update the inputs. */
  default void updateInputs(elevatorIOInputs inputs) {}

  /** Run elevator to position - Motion Magic*/
  default void movetoHeight(double posRads) {}

  /** Sets the elevator to a height, as in "resetting" the elevator */
  default void setHeight(double posRads) {}

  /** Stop slam elevator */
  default void stop() {}

  /** Enable or disable brake mode on the elevator motor. */
  default void setBrakeMode(boolean enable) {}

  /** Displays the periodically updated outtake rate on the Shuffleboard */
  public default void updateShuffleboard() {}
}
