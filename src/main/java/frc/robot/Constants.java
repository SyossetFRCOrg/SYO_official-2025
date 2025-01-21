// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  //TODO Find real values and units for the values below
  public static final class AlgaeIntakeConstants {
    public static enum AlgaeIntakePosition {
      UP,
      DOWN
    }

    public final static int ALGAE_INTAKE_PORT = 0;
    public final static double ALGAE_INTAKE_SPEED = 0.5;
    public final static double ALGAE_OUTTAKE_SPEED = -0.5;
  }
  
  public static final class Elevator {
    public static enum ElevatorPosition{
      //account for encoder offset. units in meters
      BOTTOM(0.0), //TODO
      INTAKE_PREP(0.0), //TODO
      INTAKE(0.0), //TODO
      ALGAE_L2(0.0), //TODO
      ALGAE_L3(0.0), //TODO

      L1(0.0), //TODO
      L2(0.0), //TODO
      L3(0.0), //TODO
      L4(0.0); //TODO

      public final double value;
      
      private ElevatorPosition(double value) {
      this.value = value;
    }
    }

    public static final int MOTOR_ID = 0; //TODO

    public static final double kP = 0.0; // TODO
    public static final double kI = 0.0; // TODO
    public static final double kD = 0.0; // TODO
    public static final double kS = 0.0; // TODO
    public static final double kG = 0.0; // TODO
    public static final double kV = 0.0; // TODO
    public static final double kA = 0.0; // TODO

    public static final double CURRENT_LIMIT = 0.0; //TODO
  }
}
