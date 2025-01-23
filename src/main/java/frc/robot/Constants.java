// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class ArmConstants {
    // TODO replace with real values
    public static final int ARM_WRIST_PORT = 1;

    public static enum ArmPosition {
      BOTTOM(-Math.PI / 2.0 + Units.degreesToRadians(5)),
      HORIZONTAL(0),
      L1(0),
      L2(Units.degreesToRadians(55)), // reef angle
      L3(Units.degreesToRadians(55)),
      L4(1.033),
      TOP(Math.PI / 2.0);

      public final double value;

      private ArmPosition(double value) {
        this.value = value;
      }
    }
    public static final int ARM_MOTOR_ID = 19;
    public static final int ARM_CURRENT_LIMIT = 20;
    public static final int ARM_VOLTAGE = 2;
  }
}
