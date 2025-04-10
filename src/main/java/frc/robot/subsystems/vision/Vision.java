package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final Drive drive;

  public Vision(VisionConsumer consumer, Drive drive, VisionIO... io) {
    this.consumer = consumer;
    this.drive = drive;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      List<Pose3d> robotPosesAcceptedMT1 = new LinkedList<>();
      List<Pose3d> robotPosesAcceptedMT2 = new LinkedList<>();

      List<Pose3d> robotPosesRejectedMT1 = new LinkedList<>();
      List<Pose3d> robotPosesRejectedMT2 = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        // Translation2d velocity =
        //     new Translation2d(
        //         drive.getChassisSpeeds().vxMetersPerSecond,
        //         drive.getChassisSpeeds().vyMetersPerSecond);

        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth()
                || (observation.pose().getX() == 0.0 && observation.pose().getY() == 0.0)
                || (Math.abs(drive.getChassisSpeeds().omegaRadiansPerSecond) > (Math.PI * 3.0 / 4.0)
                    && observation.type()
                        == PoseObservationType.MEGATAG_2) // reject if omega too high
                || (observation.averageTagDistance() < .45
                    && observation.type()
                        == PoseObservationType.MEGATAG_1) // instability for MT1 when too near
                || (observation.type() == PoseObservationType.MEGATAG_2
                    && DriverStation.isDisabled());

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
          if (observation.type() == PoseObservationType.MEGATAG_1) {
            robotPosesRejectedMT1.add(observation.pose());
          } else if (observation.type() == PoseObservationType.MEGATAG_2) {
            robotPosesRejectedMT2.add(observation.pose());
          }
        } else {
          robotPosesAccepted.add(observation.pose());
          if (observation.type() == PoseObservationType.MEGATAG_1) {
            robotPosesAcceptedMT1.add(observation.pose());
          } else if (observation.type() == PoseObservationType.MEGATAG_2) {
            robotPosesAcceptedMT2.add(observation.pose());
          }
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double linearstdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double thetastdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();

        // if (DriverStation.isEnabled()) {
        // The farther the current estimate is from the pose right now, the less we trust it.
        // It should still allow for correction when we are very off because of the many poses
        // coming in continuously.
        // This is meant to allow for that extreme correction (albeit slow) while making
        // occasional
        // nonsense
        // negligible.
        if (observation
                .pose()
                .toPose2d()
                .getTranslation()
                .getDistance(drive.getPose().getTranslation())
            > .5) {
          linearstdDevFactor *= 5;
        }
        if (observation
                .pose()
                .toPose2d()
                .getTranslation()
                .getDistance(drive.getPose().getTranslation())
            > 1) {
          linearstdDevFactor *= 5;
        }
        if (observation
                .pose()
                .toPose2d()
                .getTranslation()
                .getDistance(drive.getPose().getTranslation())
            > 1.5) {
          linearstdDevFactor *= 5;
        }
        if (observation
                .pose()
                .toPose2d()
                .getTranslation()
                .getDistance(drive.getPose().getTranslation())
            > 2) {
          linearstdDevFactor *= 10;
        }

        // same for rotational corrections.
        // However, also increase linear StdDev because the way MT1 works, if it returns a
        // rotation
        // that is very off, it takes the translation with it as well. This should already be
        // compensated for
        // in the translational adjustments, but this is for more safety
        if (observation
                .pose()
                .toPose2d()
                .getRotation()
                .minus(drive.getPose().getRotation())
                .getDegrees()
            > 10) {
          thetastdDevFactor *= 5;
          linearstdDevFactor *= 5;
        }
        if (observation
                .pose()
                .toPose2d()
                .getRotation()
                .minus(drive.getPose().getRotation())
                .getDegrees()
            > 15) {
          thetastdDevFactor *= 5;
          linearstdDevFactor *= 5;
        }
        if (observation
                .pose()
                .toPose2d()
                .getRotation()
                .minus(drive.getPose().getRotation())
                .getDegrees()
            > 20) {
          thetastdDevFactor *= 5;
          // linearstdDevFactor *= 5;
        }
        if (observation
                .pose()
                .toPose2d()
                .getRotation()
                .minus(drive.getPose().getRotation())
                .getDegrees()
            > 25) {
          thetastdDevFactor *= 5;
          // linearstdDevFactor *= 5;
        }
        if (observation
                .pose()
                .toPose2d()
                .getRotation()
                .minus(drive.getPose().getRotation())
                .getDegrees()
            > 30) {
          thetastdDevFactor *= 5;
          // linearstdDevFactor *= 5;
        }

        // for some reason mt2 is also somewhat jumpy near the tag, which messes up auto align
        // since
        // the
        // pose itself jumps so often. This should smooth it out while still not directly
        // neglecting
        // the new pose inputs.
        if (observation.averageTagDistance() < 1) {
          thetastdDevFactor *= 4000000;
        }
        // }

        double linearStdDev = linearStdDevBaseline * linearstdDevFactor;
        double angularStdDev = angularStdDevBaseline * thetastdDevFactor;

        // have to do standard deviation tuning originally from MT1 values.
        // then also tune it for MT2, then the ratio of MT2 stddev / MT1 stddev is the stddev factor
        // for linear
        // except for angular stddev because it just uses the gyro's angle anyway, so stddev is
        // infinite

        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }

        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        if (observation.type() == PoseObservationType.MEGATAG_1) {
          Logger.recordOutput(
              "Vision/Camera" + Integer.toString(cameraIndex) + "/MT1StdDevs",
              new double[] {linearStdDev, angularStdDev});
        }

        if (observation.type() == PoseObservationType.MEGATAG_2) {
          Logger.recordOutput(
              "Vision/Camera" + Integer.toString(cameraIndex) + "/MT2StdDevs",
              new double[] {linearStdDev, angularStdDev});
        }

        // Send vision observation only if vision is enabled
        if (RobotState.getInstance().isAddingVision()) {
          consumer.accept(
              observation.pose().toPose2d(),
              observation.timestamp(),
              VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
        }
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));

      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/MT1RobotPosesAccepted",
          robotPosesAcceptedMT1.toArray(new Pose3d[robotPosesAcceptedMT1.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/MT1RobotPosesRejected",
          robotPosesRejectedMT1.toArray(new Pose3d[robotPosesRejectedMT1.size()]));

      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/MT2RobotPosesAccepted",
          robotPosesAcceptedMT2.toArray(new Pose3d[robotPosesAcceptedMT2.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/MT2RobotPosesRejected",
          robotPosesRejectedMT2.toArray(new Pose3d[robotPosesRejectedMT2.size()]));

      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}

// package frc.robot.subsystems.vision;

// import static frc.robot.subsystems.vision.VisionConstants.*;

// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.wpilibj.Alert;
// import edu.wpi.first.wpilibj.Alert.AlertType;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
// import java.util.LinkedList;
// import java.util.List;
// import org.littletonrobotics.junction.Logger;

// public class Vision extends SubsystemBase {
//   private final VisionConsumer consumer;
//   private final VisionIO[] io;
//   private final VisionIOInputsAutoLogged[] inputs;
//   private final Alert[] disconnectedAlerts;
//   private final Drive drive;

//   public Vision(VisionConsumer consumer, Drive drive, VisionIO... io) {
//     this.consumer = consumer;
//     this.drive = drive;
//     this.io = io;

//     // Initialize inputs
//     this.inputs = new VisionIOInputsAutoLogged[io.length];
//     for (int i = 0; i < inputs.length; i++) {
//       inputs[i] = new VisionIOInputsAutoLogged();
//     }

//     // Initialize disconnected alerts
//     this.disconnectedAlerts = new Alert[io.length];
//     for (int i = 0; i < inputs.length; i++) {
//       disconnectedAlerts[i] =
//           new Alert(
//               "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
//     }
//   }

//   /**
//    * Returns the X angle to the best target, which can be used for simple servoing with vision.
//    *
//    * @param cameraIndex The index of the camera to use.
//    */
//   public Rotation2d getTargetX(int cameraIndex) {
//     return inputs[cameraIndex].latestTargetObservation.tx();
//   }

//   @Override
//   public void periodic() {
//     for (int i = 0; i < io.length; i++) {
//       io[i].updateInputs(inputs[i]);
//       Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
//     }

//     // Initialize logging values
//     List<Pose3d> allTagPoses = new LinkedList<>();
//     List<Pose3d> allRobotPoses = new LinkedList<>();
//     List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
//     List<Pose3d> allRobotPosesRejected = new LinkedList<>();

//     // Loop over cameras
//     for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
//       // Update disconnected alert
//       disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

//       // Initialize logging values
//       List<Pose3d> tagPoses = new LinkedList<>();
//       List<Pose3d> robotPoses = new LinkedList<>();
//       List<Pose3d> robotPosesAccepted = new LinkedList<>();
//       List<Pose3d> robotPosesRejected = new LinkedList<>();

//       List<Pose3d> robotPosesAcceptedMT1 = new LinkedList<>();
//       List<Pose3d> robotPosesAcceptedMT2 = new LinkedList<>();

//       List<Pose3d> robotPosesRejectedMT1 = new LinkedList<>();
//       List<Pose3d> robotPosesRejectedMT2 = new LinkedList<>();

//       // Add tag poses
//       for (int tagId : inputs[cameraIndex].tagIds) {
//         var tagPose = aprilTagLayout.getTagPose(tagId);
//         if (tagPose.isPresent()) {
//           tagPoses.add(tagPose.get());
//         }
//       }

//       // Loop over pose observations
//       for (var observation : inputs[cameraIndex].poseObservations) {
//         // Check whether to reject pose
//         Translation2d velocity =
//             new Translation2d(
//                 drive.getChassisSpeeds().vxMetersPerSecond,
//                 drive.getChassisSpeeds().vyMetersPerSecond);
//         boolean rejectPose =
//             observation.tagCount() == 0 // Must have at least one tag
//                 || (observation.tagCount() == 1
//                     && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
//                 || Math.abs(observation.pose().getZ())
//                     > maxZError // Must have realistic Z coordinate

//                 // Must be within the field boundaries
//                 || observation.pose().getX() < 0.0
//                 || observation.pose().getX() > aprilTagLayout.getFieldLength()
//                 || observation.pose().getY() < 0.0
//                 || observation.pose().getY() > aprilTagLayout.getFieldWidth()
//                 || (observation.pose().getX() == 0.0 && observation.pose().getY() == 0.0)
//                 || velocity.getNorm() > 2.5
//                 || Math.abs(drive.getChassisSpeeds().omegaRadiansPerSecond)
//                     > (2 * Math.PI) // reject if omega too high
//                 || (observation.type() == PoseObservationType.MEGATAG_1
//                     && observation.averageTagDistance() < .75);

//         // Add pose to log
//         robotPoses.add(observation.pose());
//         if (rejectPose) {
//           robotPosesRejected.add(observation.pose());
//           if (observation.type() == PoseObservationType.MEGATAG_1) {
//             robotPosesRejectedMT1.add(observation.pose());
//           } else if (observation.type() == PoseObservationType.MEGATAG_2) {
//             robotPosesRejectedMT2.add(observation.pose());
//           }
//         } else {
//           robotPosesAccepted.add(observation.pose());
//           if (observation.type() == PoseObservationType.MEGATAG_1) {
//             robotPosesAcceptedMT1.add(observation.pose());
//           } else if (observation.type() == PoseObservationType.MEGATAG_2) {
//             robotPosesAcceptedMT2.add(observation.pose());
//           }
//         }

//         // Skip if rejected
//         if (rejectPose) {
//           continue;
//         }

//         // Calculate standard deviations
//         double linearstdDevFactor =
//             Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
//         double thetastdDevFactor =
//             Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();

//         //The farther we are from the pose right now, the less we trust it.
//         //It should still allow for correction when we are very off because of the multiple poses
// coming in continuously.
//         //This is meant to allow for that extreme correction while making occasional nonsense
// negligible.
//         // if
// (observation.pose().toPose2d().getTranslation().getDistance(drive.getPose().getTranslation()) >
// .5){
//         //   linearstdDevFactor *= 1.5;
//         // }
//         // if
// (observation.pose().toPose2d().getTranslation().getDistance(drive.getPose().getTranslation()) >
// 1){
//         //   linearstdDevFactor *= 1.5;
//         // }
//         // if
// (observation.pose().toPose2d().getTranslation().getDistance(drive.getPose().getTranslation()) >
// 1.5){
//         //   linearstdDevFactor *= 1.5;
//         // }
//         // if
// (observation.pose().toPose2d().getTranslation().getDistance(drive.getPose().getTranslation()) >
// 2){
//         //   linearstdDevFactor *= 1.5;
//         // }
//         // if
// (observation.pose().toPose2d().getTranslation().getDistance(drive.getPose().getTranslation()) >
// 2.5){
//         //   linearstdDevFactor *= 1.5;
//         // }

//         // //same for rotational corrections.
//         // if
// (observation.pose().toPose2d().getRotation().minus(drive.getPose().getRotation()).getDegrees() >
// 10){
//         //   thetastdDevFactor *= 1.2;
//         // }
//         // if
// (observation.pose().toPose2d().getRotation().minus(drive.getPose().getRotation()).getDegrees() >
// 15){
//         //   thetastdDevFactor *= 1.2;
//         // }
//         // if
// (observation.pose().toPose2d().getRotation().minus(drive.getPose().getRotation()).getDegrees() >
// 20){
//         //   thetastdDevFactor *= 1.2;
//         // }
//         // if
// (observation.pose().toPose2d().getRotation().minus(drive.getPose().getRotation()).getDegrees() >
// 25){
//         //   thetastdDevFactor *= 1.2;
//         // }
//         // if
// (observation.pose().toPose2d().getRotation().minus(drive.getPose().getRotation()).getDegrees() >
// 30){
//         //   thetastdDevFactor *= 1.2;
//         // }

//         double linearStdDev = linearStdDevBaseline * linearstdDevFactor;
//         double angularStdDev = angularStdDevBaseline * thetastdDevFactor;

//         // have to do standard deviation tuning originally from MT1 values.
//         // then also tune it for MT2, then the ratio of MT2 stddev / MT1 stddev is the stddev
// factor
//         // for linear
//         // except for angular stddev because it just uses the gyro's angle anyway, so stddev is
//         // infinite
//         if (observation.type() == PoseObservationType.MEGATAG_2) {
//           linearStdDev *= linearStdDevMegatag2Factor;
//           angularStdDev *= angularStdDevMegatag2Factor;
//         }
//         if (cameraIndex < cameraStdDevFactors.length) {
//           linearStdDev *= cameraStdDevFactors[cameraIndex];
//           angularStdDev *= cameraStdDevFactors[cameraIndex];
//         }

//         // Send vision observation
//         consumer.accept(
//             observation.pose().toPose2d(),
//             observation.timestamp(),
//             VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
//       }

//       // Log camera datadata
//       Logger.recordOutput(
//           "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
//           tagPoses.toArray(new Pose3d[tagPoses.size()]));
//       Logger.recordOutput(
//           "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
//           robotPoses.toArray(new Pose3d[robotPoses.size()]));
//       Logger.recordOutput(
//           "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
//           robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
//       Logger.recordOutput(
//           "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
//           robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));

//       Logger.recordOutput(
//           "Vision/Camera" + Integer.toString(cameraIndex) + "/MT1RobotPosesAccepted",
//           robotPosesAcceptedMT1.toArray(new Pose3d[robotPosesAcceptedMT1.size()]));
//       Logger.recordOutput(
//           "Vision/Camera" + Integer.toString(cameraIndex) + "/MT1RobotPosesRejected",
//           robotPosesRejectedMT1.toArray(new Pose3d[robotPosesRejectedMT1.size()]));

//       Logger.recordOutput(
//           "Vision/Camera" + Integer.toString(cameraIndex) + "/MT2RobotPosesAccepted",
//           robotPosesAcceptedMT2.toArray(new Pose3d[robotPosesAcceptedMT2.size()]));
//       Logger.recordOutput(
//           "Vision/Camera" + Integer.toString(cameraIndex) + "/MT2RobotPosesRejected",
//           robotPosesRejectedMT2.toArray(new Pose3d[robotPosesRejectedMT2.size()]));

//       allTagPoses.addAll(tagPoses);
//       allRobotPoses.addAll(robotPoses);
//       allRobotPosesAccepted.addAll(robotPosesAccepted);
//       allRobotPosesRejected.addAll(robotPosesRejected);
//     }

//     // Log summary data
//     Logger.recordOutput(
//         "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
//     Logger.recordOutput(
//         "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
//     Logger.recordOutput(
//         "Vision/Summary/RobotPosesAccepted",
//         allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
//     Logger.recordOutput(
//         "Vision/Summary/RobotPosesRejected",
//         allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
//   }

//   @FunctionalInterface
//   public static interface VisionConsumer {
//     public void accept(
//         Pose2d visionRobotPoseMeters,
//         double timestampSeconds,
//         Matrix<N3, N1> visionMeasurementStdDevs);
//   }
// }
