// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVoltsIntake = 0.0;
    public double appliedVoltsRotate = 0.0;
    public double[] currentAmps = new double[] {};
    public boolean intaked = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}


  public default void configurePID(double kP, double kI, double kD) {}

  /** Displays the periodically updated outtake rate on the Shuffleboard */
  public default void updateShuffleboard() {}

    /** Returns the current angle of the intake (rad). */
  public default double getAngle() {
    return 0.0;
  }
  
  public default double getIntakeVelocity() {
    return 0.0;
  }

  /**
     * Engages the intake.
     * 
     * @param intakeRate The rate of intake (rpm).
     */
    public default void intake(double intakeRate) {}


    /**
   * Changes the intake angle.
   * 
   * @param angularSpeed The speed of angle change (rad/s).
   */
  public default void rotate(double angle) {}

  /** Resets the angle of the intake to 0. */
  public default void reset() {}

  /** Returns whether the intake can be activated. */
  public default boolean canIntake() { return true;}

  /** Returns whether the intake can rotate. */
  public default boolean canRotate() { return true;}

  public default void stop(boolean interrupted) {}


      
}
