// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.generated.TunerConstants;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;

import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
// import org.littletonrobotics.frc2024.subsystems.drive.DriveConstants;
// import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmConstants;
// import org.littletonrobotics.frc2024.util.AllianceFlipUtil;
// import org.littletonrobotics.frc2024.util.GeomUtil;
// import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
// import org.littletonrobotics.frc2024.util.NoteVisualizer;
// import org.littletonrobotics.frc2024.util.swerve.ModuleLimits;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

@ExtensionMethod({GeomUtil.class})
public class RobotState {

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  private Pose2d[] reefscoringPositions = //not finalized or tuned. pose2d of all the blue reef scoring positions
  { //use alliancefliputil to flip to get corresponding red scoring pose2ds
    new Pose2d(3.2292869091033936, 3.8667519092559814, Rotation2d.fromDegrees(0)),
    new Pose2d(3.2235898971557617, 4.180093288421631, Rotation2d.fromDegrees(0)),

    new Pose2d(3.707775115966797, 5.033270359039307, Rotation2d.fromRadians(-1.0466175637493382)),
    new Pose2d(3.985335350036621, 5.185656547546387, Rotation2d.fromRadians(-1.0466175637493382)),
    
    new Pose2d(4.970402717590332, 5.174771785736084, Rotation2d.fromRadians(-2.0988710476023327)),
    new Pose2d(5.264289855957031 , 5.011500835418701, Rotation2d.fromRadians(-2.0988710476023327)),

    new Pose2d(5.726890563964844, 4.184262275695801, Rotation2d.fromDegrees(180)),
    new Pose2d(5.732332706451416 , 3.868605375289917, Rotation2d.fromDegrees(180)),
    
  };
  private RobotState(){

  }


  
  public double getDistanceToNearestReef(Pose2d pose){
    double mindistance = Double.POSITIVE_INFINITY;
      int index = -1;
      for (int i=0; i < reefscoringPositions.length; i++){
        if (pose.getTranslation().getDistance(reefscoringPositions[i].getTranslation()) < mindistance)
        {
          index = i;
          mindistance = pose.getTranslation().getDistance(reefscoringPositions[i].getTranslation());
        }
      }

      return mindistance;
  }

  @AutoLogOutput(key = "NearestReefPose")
  public Pose2d getNearestReefPose(Pose2d pose){
    double mindistance = Double.POSITIVE_INFINITY;
      int index = -1;
      for (int i=0; i < reefscoringPositions.length; i++){
        if (pose.getTranslation().getDistance(reefscoringPositions[i].getTranslation()) < mindistance)
        {
          index = i;
          mindistance = pose.getTranslation().getDistance(reefscoringPositions[i].getTranslation());
        }
      }

      return reefscoringPositions[index];
  }
  public PathPlannerPath getPathToNearestReef(Pose2d pose){
    
      double mindistance = Double.POSITIVE_INFINITY;
      int index = -1;
      for (int i=0; i < reefscoringPositions.length; i++){
        if (pose.getTranslation().getDistance(reefscoringPositions[i].getTranslation()) < mindistance)
        {
          index = i;
          mindistance = pose.getTranslation().getDistance(reefscoringPositions[i].getTranslation());
        }
      }
      
      /**
       * The waypointsFromPoses method required that the rotation component 
       * of each pose is the direction of travel, not the rotation of a swerve chassis.
       *
       * To set the rotation the path should end with, use the GoalEndState.
       */
  
       //if this works i'm gonna go crazy
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
          new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(0)),
          new Pose2d(pose.getTranslation().interpolate(reefscoringPositions[index].getTranslation(), 0.5), reefscoringPositions[index].getTranslation().minus(pose.getTranslation()).getAngle()),
          reefscoringPositions[index]
      );

      // List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      //   new Pose2d(0,4,Rotation2d.fromDegrees(0)),
      //   new Pose2d(2,4,Rotation2d.fromDegrees(0))
      //   //,
      //   // new Pose2d(4,4,Rotation2d.fromDegrees(0))
      //   );



  
      PathConstraints constraints = new PathConstraints(TunerConstants.moduleLimitsFree.maxDriveVelocity() * .8, TunerConstants.moduleLimitsFree.maxDriveAcceleration() * .8, TunerConstants.moduleLimitsFree.maxSteeringVelocity() * .8, TunerConstants.moduleLimitsFree.maxSteeringVelocity() * 1.5); // The constraints for this path.
      // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
  
      // Create the path using the waypoints created above
      PathPlannerPath path = new PathPlannerPath(
              waypoints,
              constraints,
              null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
              new GoalEndState(0.0, reefscoringPositions[index].getRotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
      );
      
  
      // Prevent the path from being flipped if the coordinates are already correct
      path.preventFlipping = true;
      
    return path;
  }



  // private RobotState() {
  //   for (int i = 0; i < 3; ++i) {
  //     qStdDevs.set(i, 0, Math.pow(DriveConstants.odometryStateStdDevs.get(i, 0), 2));
  //   }
  //   kinematics = DriveConstants.kinematics;

  //   // Setup NoteVisualizer
  //   NoteVisualizer.setRobotPoseSupplier(this::getEstimatedPose);
  // }

  // /** Add odometry observation */
  // public void addOdometryObservation(OdometryObservation observation) {
  //   latestParameters = null;
  //   latestSuperPoopParameters = null;
  //   Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
  //   lastWheelPositions = observation.wheelPositions();
  //   // Check gyro connected
  //   if (observation.gyroAngle != null) {
  //     // Update dtheta for twist if gyro connected
  //     twist =
  //         new Twist2d(
  //             twist.dx, twist.dy, observation.gyroAngle().minus(lastGyroAngle).getRadians());
  //     lastGyroAngle = observation.gyroAngle();
  //   }
  //   // Add twist to odometry pose
  //   odometryPose = odometryPose.exp(twist);
  //   // Add pose to buffer at timestamp
  //   poseBuffer.addSample(observation.timestamp(), odometryPose);
  //   // Calculate diff from last odometry pose and add onto pose estimate
  //   estimatedPose = estimatedPose.exp(twist);
  // }

  // public void addVisionObservation(VisionObservation observation) {
  //   latestParameters = null;
  //   latestSuperPoopParameters = null;
  //   // If measurement is old enough to be outside the pose buffer's timespan, skip.
  //   try {
  //     if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSeconds
  //         > observation.timestamp()) {
  //       return;
  //     }
  //   } catch (NoSuchElementException ex) {
  //     return;
  //   }
  //   // Get odometry based pose at timestamp
  //   var sample = poseBuffer.getSample(observation.timestamp());
  //   if (sample.isEmpty()) {
  //     // exit if not there
  //     return;
  //   }

  //   // sample --> odometryPose transform and backwards of that
  //   var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
  //   var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
  //   // get old estimate by applying odometryToSample Transform
  //   Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

  //   // Calculate 3 x 3 vision matrix
  //   var r = new double[3];
  //   for (int i = 0; i < 3; ++i) {
  //     r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
  //   }
  //   // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
  //   // and C = I. See wpimath/algorithms.md.
  //   Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
  //   for (int row = 0; row < 3; ++row) {
  //     double stdDev = qStdDevs.get(row, 0);
  //     if (stdDev == 0.0) {
  //       visionK.set(row, row, 0.0);
  //     } else {
  //       visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
  //     }
  //   }
  //   // difference between estimate and vision pose
  //   Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose());
  //   // scale transform by visionK
  //   var kTimesTransform =
  //       visionK.times(
  //           VecBuilder.fill(
  //               transform.getX(), transform.getY(), transform.getRotation().getRadians()));
  //   Transform2d scaledTransform =
  //       new Transform2d(
  //           kTimesTransform.get(0, 0),
  //           kTimesTransform.get(1, 0),
  //           Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

  //   // Recalculate current estimate by applying scaled transform to old estimate
  //   // then replaying odometry data
  //   estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
  // }

  // public void addVelocityData(Twist2d robotVelocity) {
  //   latestParameters = null;
  //   this.robotVelocity = robotVelocity;
  // }

  // public void addTrajectoryVelocityData(Twist2d robotVelocity) {
  //   latestParameters = null;
  //   trajectoryVelocity = robotVelocity;
  // }

  // public AimingParameters getAimingParameters() {
  //   if (latestParameters != null) {
  //     // Cache previously calculated aiming parameters. Cache is invalidated whenever new
  //     // observations are added.
  //     return latestParameters;
  //   }

  //   Transform2d fieldToTarget =
  //       AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening)
  //           .toTranslation2d()
  //           .toTransform2d()
  //           .plus(FudgeFactors.speaker.getTransform());
  //   Pose2d fieldToPredictedVehicle;
  //   if (DriverStation.isAutonomousEnabled()) {
  //     fieldToPredictedVehicle = getPredictedPose(autoLookahead.get(), autoLookahead.get());

  //   } else {
  //     fieldToPredictedVehicle =
  //         lookaheadDisable.getAsBoolean()
  //             ? getEstimatedPose()
  //             : getPredictedPose(lookahead.get(), lookahead.get());
  //   }
  //   Logger.recordOutput("RobotState/AimingParameters/PredictedPose", fieldToPredictedVehicle);

  //   Pose2d fieldToPredictedVehicleFixed =
  //       new Pose2d(fieldToPredictedVehicle.getTranslation(), new Rotation2d());

  //   Translation2d predictedVehicleToTargetTranslation =
  //       fieldToPredictedVehicle.inverse().transformBy(fieldToTarget).getTranslation();
  //   Translation2d predictedVehicleFixedToTargetTranslation =
  //       fieldToPredictedVehicleFixed.inverse().transformBy(fieldToTarget).getTranslation();

  //   Rotation2d targetVehicleDirection = predictedVehicleFixedToTargetTranslation.getAngle();
  //   double targetDistance = predictedVehicleToTargetTranslation.getNorm();

  //   double armAngleDegrees = armAngleCoefficient * Math.pow(targetDistance, armAngleExponent);
  //   double autoFarArmCorrection =
  //       DriverStation.isAutonomousEnabled() && targetDistance >= Units.inchesToMeters(125)
  //           ? autoFarShotCompensationDegrees
  //           : 0.0;
  //   Logger.recordOutput(
  //       "RobotState/AimingParameters/AutoFarArmCorrectionDegrees", autoFarArmCorrection);
  //   latestParameters =
  //       new AimingParameters(
  //           targetVehicleDirection,
  //           Rotation2d.fromDegrees(
  //               armAngleDegrees + shotCompensationDegrees + autoFarArmCorrection),
  //           targetDistance,
  //           new FlywheelSpeeds(0, 0));
  //   return latestParameters;
  // }

  // private static final Translation2d superPoopTarget =
  //     FieldConstants.Subwoofer.centerFace
  //         .getTranslation()
  //         .interpolate(FieldConstants.ampCenter, 0.5);

  // public AimingParameters getSuperPoopAimingParameters() {
  //   if (latestSuperPoopParameters != null) {
  //     return latestSuperPoopParameters;
  //   }
  //   Pose2d predictedFieldToRobot =
  //       getPredictedPose(superPoopLookahead.get(), superPoopLookahead.get());
  //   Translation2d predictedRobotToTarget =
  //       AllianceFlipUtil.apply(superPoopTarget).minus(predictedFieldToRobot.getTranslation());
  //   double effectiveDistance = predictedRobotToTarget.getNorm();
  //   var flywheelSpeeds = superPoopFlywheelSpeedsMap.get(effectiveDistance);
  //   var armAngle = Rotation2d.fromDegrees(superPoopArmAngleMap.get(effectiveDistance));

  //   Translation2d vehicleVelocity =
  //       new Translation2d(robotVelocity.dx, robotVelocity.dy)
  //           .rotateBy(predictedRobotToTarget.getAngle().unaryMinus());
  //   Logger.recordOutput("RobotState/SuperPoopParameters/RadialVelocity", vehicleVelocity.getX());
  //   double radialVelocity =
  //       Units.radiansPerSecondToRotationsPerMinute(
  //               vehicleVelocity.getX() / Units.inchesToMeters(1.5))
  //           * armAngle.getCos();
  //   flywheelSpeeds =
  //       new FlywheelSpeeds(
  //           flywheelSpeeds.leftSpeed() - radialVelocity,
  //           flywheelSpeeds.rightSpeed() - radialVelocity);

  //   latestSuperPoopParameters =
  //       new AimingParameters(
  //           predictedRobotToTarget.getAngle(), armAngle, effectiveDistance, flywheelSpeeds);
  //   return latestSuperPoopParameters;
  // }

  // public void setDemoTagPose(Pose3d demoTagPose) {
  //   this.demoTagPose = demoTagPose;
  //   latestDemoParamters = null;
  // }

  // private static final LoggedTunableNumber demoTargetDistance =
  //     new LoggedTunableNumber("RobotState/DemoTargetDistance", 2.0);

  // public Optional<DemoFollowParameters> getDemoTagParameters() {
  //   if (latestDemoParamters != null) {
  //     // Use cached demo parameters.
  //     return Optional.of(latestDemoParamters);
  //   }
  //   // Return empty optional if no demo tag pose.
  //   if (demoTagPose == null) return Optional.empty();

  //   // Calculate target pose.
  //   Pose2d targetPose =
  //       demoTagPose
  //           .toPose2d()
  //           .transformBy(
  //               new Transform2d(
  //                   new Translation2d(demoTargetDistance.get(), 0.0), new Rotation2d(Math.PI)));

  //   // Calculate heading without movement.
  //   Translation2d demoTagFixed = demoTagPose.getTranslation().toTranslation2d();
  //   Translation2d robotToDemoTagFixed = demoTagFixed.minus(getEstimatedPose().getTranslation());
  //   Rotation2d targetHeading = robotToDemoTagFixed.getAngle();

  //   // Calculate arm angle.
  //   double z = demoTagPose.getZ();
  //   Rotation2d armAngle =
  //       new Rotation2d(
  //           robotToDemoTagFixed.getNorm() - ArmConstants.armOrigin.getX(),
  //           z - ArmConstants.armOrigin.getY());

  //   latestDemoParamters = new DemoFollowParameters(targetPose, targetHeading, armAngle);
  //   return Optional.of(latestDemoParamters);
  // }

  // public ModuleLimits getModuleLimits() {
  //   return flywheelAccelerating && !DriverStation.isAutonomousEnabled()
  //       ? DriveConstants.moduleLimitsFlywheelSpinup
  //       : DriveConstants.moduleLimitsFree;
  // }

  // public boolean inShootingZone() {
  //   Pose2d robot = AllianceFlipUtil.apply(getEstimatedPose());
  //   if (robot.getY() <= FieldConstants.Stage.ampLeg.getY()) {
  //     return robot.getX() <= FieldConstants.wingX;
  //   } else {
  //     return robot.getX() <= FieldConstants.fieldLength / 2.0 + 0.5;
  //   }
  // }

  // public boolean inCloseShootingZone() {
  //   return getEstimatedPose()
  //           .getTranslation()
  //           .getDistance(
  //               AllianceFlipUtil.apply(
  //                   FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()))
  //       < Units.feetToMeters(closeShootingZoneFeet.get());
  // }

  // /**
  //  * Reset estimated pose and odometry pose to pose <br>
  //  * Clear pose buffer
  //  */
  // public void resetPose(Pose2d initialPose) {
  //   estimatedPose = initialPose;
  //   odometryPose = initialPose;
  //   poseBuffer.clear();
  // }

  // @AutoLogOutput(key = "RobotState/FieldVelocity")
  // public Twist2d fieldVelocity() {
  //   Translation2d linearFieldVelocity =
  //       new Translation2d(robotVelocity.dx, robotVelocity.dy).rotateBy(estimatedPose.getRotation());
  //   return new Twist2d(
  //       linearFieldVelocity.getX(), linearFieldVelocity.getY(), robotVelocity.dtheta);
  // }

  // @AutoLogOutput(key = "RobotState/EstimatedPose")
  // public Pose2d getEstimatedPose() {
  //   return estimatedPose;
  // }

  // /**
  //  * Predicts what our pose will be in the future. Allows separate translation and rotation
  //  * lookaheads to account for varying latencies in the different measurements.
  //  *
  //  * @param translationLookaheadS The lookahead time for the translation of the robot
  //  * @param rotationLookaheadS The lookahead time for the rotation of the robot
  //  * @return The predicted pose.
  //  */
  // public Pose2d getPredictedPose(double translationLookaheadS, double rotationLookaheadS) {
  //   Twist2d velocity = DriverStation.isAutonomousEnabled() ? trajectoryVelocity : robotVelocity;
  //   return getEstimatedPose()
  //       .transformBy(
  //           new Transform2d(
  //               velocity.dx * translationLookaheadS,
  //               velocity.dy * translationLookaheadS,
  //               Rotation2d.fromRadians(velocity.dtheta * rotationLookaheadS)));
  // }

  // @AutoLogOutput(key = "RobotState/OdometryPose")
  // public Pose2d getOdometryPose() {
  //   return odometryPose;
  // }
}
