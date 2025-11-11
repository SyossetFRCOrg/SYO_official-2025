package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class ReefAlignController {
  private static final LoggedTunableNumber linearkP =
      new LoggedTunableNumber("AutoAlign/drivekP", .85);
  private static final LoggedTunableNumber linearkD =
      new LoggedTunableNumber("AutoAlign/drivekD", 0.0);

  private static final LoggedTunableNumber linearkI =
      new LoggedTunableNumber("AutoAlign/drivekI", 10);

  private static final LoggedTunableNumber thetakP =
      new LoggedTunableNumber("AutoAlign/thetakP", 4.5);
  private static final LoggedTunableNumber thetakD =
      new LoggedTunableNumber("AutoAlign/thetakD", 0.5);
  private static final LoggedTunableNumber linearTolerance =
      new LoggedTunableNumber("AutoAlign/controllerLinearTolerance", .09);
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("AutoAlign/controllerThetaTolerance", Units.degreesToRadians(2));
  private static final LoggedTunableNumber toleranceTime =
      new LoggedTunableNumber("AutoAlign/controllerToleranceSecs", 0.25);
  //   private static final LoggedTunableNumber maxLinearVelocity =
  //       new LoggedTunableNumber(
  //           "AutoAlign/maxLinearVelocity", TunerConstants.driveConfig.maxLinearVelocity());
  //   private static final LoggedTunableNumber maxLinearAcceleration =
  //       new LoggedTunableNumber(
  //           "AutoAlign/maxLinearAcceleration",
  // TunerConstants.driveConfig.maxLinearAcceleration());
  private static final LoggedTunableNumber maxAngularVelocity =
      new LoggedTunableNumber(
          "AutoAlign/maxAngularVelocity", TunerConstants.driveConfig.maxAngularVelocity());
  private static final LoggedTunableNumber maxAngularAcceleration =
      new LoggedTunableNumber(
          "AutoAlign/maxAngularAcceleration",
          TunerConstants.driveConfig.maxAngularAcceleration() * .8);
  //   private static final LoggedTunableNumber slowLinearVelocity =
  //       new LoggedTunableNumber("AutoAlign/slowLinearVelocity",
  //       TunerConstants.driveConfig.maxLinearVelocity() * .4);
  //   private static final LoggedTunableNumber slowLinearAcceleration =
  //       new LoggedTunableNumber("AutoAlign/slowLinearAcceleration",
  //       TunerConstants.driveConfig.maxLinearAcceleration() * .7);
  //   private static final LoggedTunableNumber slowAngularVelocity =
  //       new LoggedTunableNumber("AutoAlign/slowAngularVelocity",
  //       TunerConstants.driveConfig.maxAngularVelocity() * 0.4);
  //   private static final LoggedTunableNumber slowAngularAcceleration =
  //       new LoggedTunableNumber("AutoAlign/slowAngularAcceleration",
  //       TunerConstants.driveConfig.maxAngularAcceleration() * 0.8);
  private static final LoggedTunableNumber ffMinRadius =
      new LoggedTunableNumber("AutoAlign/ffMinRadius", 0.4);
  private static final LoggedTunableNumber ffMaxRadius =
      new LoggedTunableNumber("AutoAlign/ffMaxRadius", 0.8);

  private final Pose2d desiredPose;
  private final Drive drive;
  private final Supplier<Translation2d> feedforwardSupplier;
  private final BooleanSupplier slowMode;
  private final BooleanSupplier toggle;
  private Translation2d lastSetpointTranslation;

  // Controllers for translation and rotation
  //   private PIDController linearPID;
  private ProfiledPIDController linearController;
  //   private TrapezoidProfile.State linearSetpoint;

  //   private PIDController thetaPID;
  private ProfiledPIDController thetaController;
  //   private TrapezoidProfile.State thetaSetpoint;

  private final Timer toleranceTimer = new Timer();

  public ReefAlignController(
      Drive drive,
      //  Supplier<Translation2d> feedforwardSupplier,
      BooleanSupplier slowMode,
      BooleanSupplier toggle) {
    this.drive = drive;
    this.toggle = toggle;
    this.desiredPose = RobotState.getInstance().getNearestReefPose(drive.getPose(), toggle);

    this.feedforwardSupplier = () -> new Translation2d();
    this.slowMode = slowMode;
    // Set up both controllers

    linearController =
        new ProfiledPIDController(
            linearkP.get(), 0, linearkD.get(), new TrapezoidProfile.Constraints(0, 0));
    linearController.setTolerance(linearTolerance.get());
    thetaController =
        new ProfiledPIDController(
            thetakP.get(), 0, thetakD.get(), new TrapezoidProfile.Constraints(0, 0));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(thetaTolerance.get());
    toleranceTimer.restart();
    updateConstraints();
    resetControllers();
  }

  public ReefAlignController(
      Drive drive,
      Supplier<Translation2d> feedforwardSupplier,
      BooleanSupplier slowMode,
      BooleanSupplier toggle) {
    this.drive = drive;
    this.toggle = toggle;
    this.desiredPose = RobotState.getInstance().getNearestReefPose(drive.getPose(), toggle);

    this.feedforwardSupplier = feedforwardSupplier;
    this.slowMode = slowMode;
    // Set up both controllers

    linearController =
        new ProfiledPIDController(
            linearkP.get(), 0, linearkD.get(), new TrapezoidProfile.Constraints(0, 0));
    linearController.setTolerance(linearTolerance.get());
    thetaController =
        new ProfiledPIDController(
            thetakP.get(), 0, thetakD.get(), new TrapezoidProfile.Constraints(0, 0));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(thetaTolerance.get());
    toleranceTimer.restart();
    updateConstraints();
    resetControllers();
  }

  private void updateConstraints() {

    if (drive.getPose().getTranslation().getDistance(desiredPose.getTranslation()) < .1) {

      linearController.setI(linearkI.get() * 10);
      linearController.setP(linearkP.get() * 5); // to be tuned

      linearController.setD(linearkD.get() * 5);
    }
    if (drive.getPose().getTranslation().getDistance(desiredPose.getTranslation()) > .1) {

      linearController.setI(linearkI.get());
      linearController.setP(linearkP.get() * 2.2); // to be tuned

      linearController.setD(linearkD.get() * 2.1);
    }

    if (slowMode.getAsBoolean()) {
      //   linearController.setConstraints(
      //       new TrapezoidProfile.Constraints(slowLinearVelocity.get(),
      // slowLinearAcceleration.get()));
      //   thetaController.setConstraints(
      //       new TrapezoidProfile.Constraints(
      //           slowAngularVelocity.get(), slowAngularAcceleration.get()));
      //   linearController.setPID(linearkP.get() * 2.2, linearkI.get(), linearkD.get() * 2.1);

      //   linearController.setIZone(0.2);

    } else {
      linearController.setConstraints(
          new TrapezoidProfile.Constraints(
              RobotState.getInstance().getModuleLimits().maxDriveVelocity(),
              RobotState.getInstance().getModuleLimits().maxDriveAcceleration()));
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(maxAngularVelocity.get(), maxAngularAcceleration.get()));
      linearController.setPID(linearkP.get(), linearkI.get(), linearkD.get());
      linearController.setIZone(0.2);
    }
  }

  private void resetControllers() {
    // Reset measurements and velocities
    Pose2d currentPose = drive.getPose();
    Pose2d goalPose = desiredPose;
    ChassisSpeeds fieldVelocity =
        (ChassisSpeeds.fromRobotRelativeSpeeds((drive.getChassisSpeeds()), drive.getRotation()));
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);

    // Rotation2d robotToGoalAngle =
    //     goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
    double linearVelocity = linearFieldVelocity.getNorm();
    // This one works but not the best
    // linearController.reset(currentPose.getTranslation().getDistance(goalPose.getTranslation()),
    // 0);

    // Mechanical Advantage's 2025 approach for their auto align. It may work?
    linearController.reset(
        currentPose.getTranslation().getDistance(goalPose.getTranslation()),
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(
                    goalPose
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));

    thetaController.reset(currentPose.getRotation().getRadians());
    lastSetpointTranslation = currentPose.getTranslation();
  }

  public Supplier<ChassisSpeeds> update() {
    // Update Controllers
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> linearController.setPID(linearkP.get(), 0, linearkD.get()),
        linearkP,
        linearkD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> thetaController.setPID(thetakP.get(), 0, thetakD.get()),
        thetakP,
        thetakD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> linearController.setTolerance(linearTolerance.get()), linearTolerance);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> thetaController.setTolerance(thetaTolerance.get()), thetaTolerance);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        this::updateConstraints,
        // maxLinearVelocity,
        // maxLinearAcceleration,
        // slowLinearVelocity,
        // slowLinearAcceleration,
        maxAngularVelocity,
        maxAngularAcceleration
        // slowAngularVelocity,
        // slowAngularAcceleration
        );

    // Control to setpoint
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = desiredPose;

    // Calculate drive speed
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScaler = // 1.0;
        MathUtil.clamp(
            (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()), 0, 1);

    linearController.reset(
        linearController.getSetpoint().position, linearController.getSetpoint().velocity);
    // thetaController.reset(thetaController.getSetpoint().position,
    // thetaController.getSetpoint().velocity);

    double driveVelocityScalar =
        linearController.getSetpoint().velocity * ffScaler
            + linearController.calculate(currentDistance, 0.0);

    if (linearController.atGoal()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(linearController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        // thetaController.getSetpoint().velocity * ffScaler
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    if (thetaController.atGoal()) thetaVelocity = 0.0;

    // Reset tolerance timer
    if (!linearController.atGoal() || !thetaController.atGoal()) {
      toleranceTimer.reset();
    }

    // Log data

    Logger.recordOutput("AutoAlign/MaxVel", linearController.getConstraints().maxVelocity);
    Logger.recordOutput("AutoAlign/MaxAccel", linearController.getConstraints().maxAcceleration);

    Logger.recordOutput("AutoAlign/DistanceMeasured", currentDistance);
    Logger.recordOutput("AutoAlign/DistanceSetpoint", linearController.getSetpoint().position);

    Logger.recordOutput("AutoAlign/VelocitySetpoint", driveVelocityScalar);
    Logger.recordOutput(
        "AutoAlign/'calculate'velocitySetPoint", linearController.calculate(currentDistance, 0.0));

    Logger.recordOutput("AutoAlign/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("AutoAlign/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "AutoAlign/SetpointPose",
        new Pose2d(
            lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
    Logger.recordOutput("Odometry/GoalPose", targetPose);
    // Logger.recordOutput("AutoAlign/AtGoal", atGoal());
    Logger.recordOutput(
        "AutoAlign/AtGoalCondition", (!linearController.atGoal() || !thetaController.atGoal()));
    Logger.recordOutput("AutoAlign/ToleranceTimer", toleranceTimer.get());

    // Command speeds
    var driveVelocity =
        new Pose2d(
                new Translation2d(0, 0),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation()
            .plus(feedforwardSupplier.get());
    final double finalThetaVelocity = thetaVelocity;

    updateConstraints();
    return () ->
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(),
            driveVelocity.getY(),
            finalThetaVelocity,
            currentPose.getRotation());
  }

  @AutoLogOutput(key = "AutoAlign/AtGoal")
  public boolean atGoal() {
    // Logger.recordOutput("AutoAlign/AtGoal", toleranceTimer.get() > toleranceTime.get());
    return toleranceTimer.get() > toleranceTime.get();
  }
}