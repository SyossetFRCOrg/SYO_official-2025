package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;

// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "limelight-threeg";

  public static String camera1Name = "limelight-three";

  public static String camera2Name = "limelight-backll";

  //   // Robot to camera transforms
  //   // (Not used by Limelight, configure in web UI instead)
  //   public static Transform3d robotToCamera0 =
  //       new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
  //   public static Transform3d robotToCamera1 =
  //       new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.5;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = Units.degreesToRadians(20); // in Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0 (LL3g)
        1.0, // Camera 1 (LL3)
        2.0, // Camera 2 (LL2+)
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available for MT2


  // Difference multipliers
  public static double[] linearThresholds = {0.5, 1.0, 1.5, 2.0};
  public static double[] linearMultipliers = {5, 5, 5, 10};

  public static double[] angularThresholds = {10, 15, 20, 25, 30};
  public static double[] angularMultipliers = {5, 5, 5, 5, 5};
  public static double[] angularLinearMultipliers = {5, 5, 1, 1, 1};

}


