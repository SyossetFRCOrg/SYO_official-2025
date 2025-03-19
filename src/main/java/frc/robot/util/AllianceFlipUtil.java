package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

// import org.littletonrobotics.frc2025.FieldConstants;
// import
// org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.ModuleForce;
// import
// org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.VehicleState;

public class AllianceFlipUtil {

  public static double applyX(double x) {
    return shouldFlip() ? Units.inchesToMeters(690.876) - x : x;
  }

  public static double applyY(double y) {
    return shouldFlip() ? Units.inchesToMeters(317) - y : y;
  }

  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  public static Rotation2d forcedApply(Rotation2d rotation) {
    return rotation.rotateBy(Rotation2d.kPi);
  }

  public static double forcedApplyX(double x) {
    return Units.inchesToMeters(690.876) - x;
  }

  public static double forcedApplyY(double y) {
    return Units.inchesToMeters(317) - y;
  }

  public static Translation2d forcedApply(Translation2d translation) {
    return new Translation2d(forcedApplyX(translation.getX()), forcedApplyY(translation.getY()));
  }

  public static Pose2d forcedApply(Pose2d pose) {
    return new Pose2d(forcedApply(pose.getTranslation()), forcedApply(pose.getRotation()));
  }

  // public static VehicleState apply(VehicleState state) {
  //   return shouldFlip()
  //       ? VehicleState.newBuilder()
  //           .setX(applyX(state.getX()))
  //           .setY(applyY(state.getY()))
  //           .setTheta(apply(Rotation2d.fromRadians(state.getTheta())).getRadians())
  //           .setVx(-state.getVx())
  //           .setVy(-state.getVy())
  //           .setOmega(state.getOmega())
  //           .addAllModuleForces(
  //               state.getModuleForcesList().stream()
  //                   .map(
  //                       forces ->
  //                           ModuleForce.newBuilder()
  //                               .setFx(-forces.getFx())
  //                               .setFy(-forces.getFy())
  //                               .build())
  //                   .toList())
  //           .build()
  //       : state;
  // }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }
}
