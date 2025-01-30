// Copyright 2021-2025 FRC 6328
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

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private double latencySubscriber;
  private double txSubscriber;
  private double tySubscriber;
  private Optional<PoseEstimate> megatag1Subscriber = Optional.empty();
  private Optional<PoseEstimate> megatag2Subscriber = Optional.empty();;

  private final String name;
  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Lmelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    this.name = name;
    var table = LimelightHelpers.getLimelightNTTable(name);
    this.rotationSupplier = rotationSupplier;
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = LimelightHelpers.getLatency_Pipeline(name);
    txSubscriber = LimelightHelpers.getTX(name);
    tySubscriber = LimelightHelpers.getTY(name);
    megatag1Subscriber = Optional.of((LimelightHelpers.getBotPoseEstimate_wpiBlue(name)));
    // megatag2Subscriber =
    //     table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
    megatag2Subscriber = Optional.of(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name));
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {

    latencySubscriber = LimelightHelpers.getLatency_Pipeline(name);
    txSubscriber = LimelightHelpers.getTX(name);
    tySubscriber = LimelightHelpers.getTY(name);

    
    megatag1Subscriber = Optional.of(LimelightHelpers.getBotPoseEstimate_wpiBlue(name));
    // megatag2Subscriber =
    //     table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
    megatag2Subscriber = Optional.of(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name));
    // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber) / 1000) < 250;

    // Update target observation
    inputs.latestTargetObservation =

        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber), Rotation2d.fromDegrees(tySubscriber));

    // Update orientation for MegaTag 2
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault()
        .flush(); // Increases network traffic but recommended by Limelight

    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    
    
      // if (megatag1Subscriber.tagCount ==0) //continue;
      
      // for (int i = 11; i < rawSample.value.length; i += 7) {
      //   tagIds.add((int) rawSample.value[i]);
      // }
      if (megatag1Subscriber.isPresent()){
      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              megatag1Subscriber.get().timestampSeconds - megatag1Subscriber.get().latency * 1.0e-3,

              // 3D pose estimate
              new Pose3d(megatag1Subscriber.get().pose),

              
              megatag1Subscriber.get().rawFiducials[0].ambiguity,

              // Tag count
              megatag1Subscriber.get().tagCount,

              // Average tag distance
              megatag1Subscriber.get().avgTagDist,

              // Observation type
              PoseObservationType.MEGATAG_1));
      }
    
      // if (megatag2Subscriber.tagSpan == 0) continue;
      
      // for (int i = 11; i < rawSample.value.length; i += 7) {
      //   tagIds.add((int) rawSample.value[i]);
      // }
      if(megatag2Subscriber.isPresent()){
      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              megatag2Subscriber.get().timestampSeconds - megatag2Subscriber.get().latency * 1.0e-3,

              // 3D pose estimate
              new Pose3d(megatag2Subscriber.get().pose),

              // Ambiguity, zeroed because the pose is already disambiguated
              0.0,

              // Tag count
              megatag2Subscriber.get().tagCount,

              // Average tag distance
              megatag2Subscriber.get().avgTagDist,

              // Observation type
              PoseObservationType.MEGATAG_2));
      }

    if (megatag1Subscriber.isPresent() || megatag2Subscriber.isPresent()){
    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
