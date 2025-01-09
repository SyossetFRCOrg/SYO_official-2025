package frc.robot;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;
import com.techhounds.houndutil.houndlog.loggers.StructArrayLogItem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@LoggedObject
public class CoralSim extends SubsystemBase {
    @Log
    private Pose3d pose = new Pose3d(-1000, -1000, -1000, new Rotation3d());
    private Pose3d[] scorePoses = new Pose3d[] {};

    private Set<CoralSimScoreLocation> scoreLocations = new HashSet<>();

    private Supplier<Pose2d> drivetrainPoseSupplier;
    private Supplier<Pose3d> relativeClawPoseSupplier;

    public enum CoralSimLocation {
        INTAKE,
        CLAW,
        HIDDEN;
    }

    public enum CoralSimScoreLocation {
        G_L4(new Pose3d(5.27, 3.86, 1.75, new Rotation3d(0, Math.PI / 2.0, 0))),
        D_L4(new Pose3d(4.24, 3.27, 1.75, new Rotation3d(0, Math.PI / 2.0, 0))),
        C_L4(new Pose3d(3.96, 3.43, 1.75, new Rotation3d(0, Math.PI / 2.0, 0)));

        public final Pose3d pose;

        CoralSimScoreLocation(Pose3d pose) {
            this.pose = pose;
        }
    }

    private CoralSimLocation location = CoralSimLocation.CLAW;
    private Timer timer = new Timer();

    public CoralSim(Supplier<Pose2d> drivetrainPoseSupplier, Supplier<Pose3d> relativeClawPoseSupplier) {

        this.drivetrainPoseSupplier = drivetrainPoseSupplier;
        this.relativeClawPoseSupplier = relativeClawPoseSupplier;

        LoggingManager.getInstance()
                .addLogger(new StructArrayLogItem<>("coralSim/scorePoses", Pose3d.struct, () -> scorePoses));
    }

    @Override
    public void periodic() {
        switch (location) {
            case CLAW -> {
                Pose3d relativeCoralPose = relativeClawPoseSupplier.get()
                        .plus(new Transform3d(0.143, 0, 0, new Rotation3d()));
                pose = new Pose3d(drivetrainPoseSupplier.get())
                        .plus(new Transform3d(relativeCoralPose.getTranslation(), relativeCoralPose.getRotation()))
                        .plus(new Transform3d(0, 0, 0, new Rotation3d(0, Math.PI / 2.0, 0)));
            }
            case INTAKE -> {
                Pose2d drivetrainPose = drivetrainPoseSupplier.get();
                Pose3d topPose = new Pose3d(drivetrainPose)
                        .plus(new Transform3d(-0.26, 0, 0.69, new Rotation3d(0, Math.PI / 4.0, 0)));
                Pose3d bottomPose = new Pose3d(drivetrainPoseSupplier.get())
                        .plus(new Transform3d(-0.04, 0, 0.45, new Rotation3d(0, Math.PI / 4.0, 0)));
                Pose3d inRobotPose = new Pose3d(drivetrainPoseSupplier.get())
                        .plus(new Transform3d(0.232, 0, 0.23, new Rotation3d()));
                double time = timer.get();
                if (time < 0.25) {
                    pose = topPose.interpolate(bottomPose, time * 4);
                } else if (time < 0.5) {
                    pose = bottomPose.interpolate(inRobotPose, (time - 0.25) * 4);
                } else {
                    pose = inRobotPose;
                }
            }
            default -> {
                pose = new Pose3d(-1000, -1000, -1000, new Rotation3d());
            }
        }

        this.scorePoses = this.scoreLocations.stream()
                .map((s) -> s.pose)
                .toArray(Pose3d[]::new);
    }

    public Command setLocationCommand(CoralSimLocation location) {
        return Commands.runOnce(() -> {
            this.location = location;
            timer.reset();
            timer.start();
        });
    }

    public Command addScoringLocationCommand(CoralSimScoreLocation location) {
        return Commands.runOnce(() -> {
            this.scoreLocations.add(location);
        });
    }

    public Command removeScoringLocationCommand(CoralSimScoreLocation location) {
        return Commands.runOnce(() -> {
            this.scoreLocations.remove(location);
        });
    }

}
