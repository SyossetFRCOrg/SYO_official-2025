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
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class AutonReefAlignController {
  private static final LoggedTunableNumber linearkP =
      new LoggedTunableNumber("AutonAlign/drivekP", .95);
  private static final LoggedTunableNumber linearkD =
      new LoggedTunableNumber("AutonAlign/drivekD", 0.0);

  private static final LoggedTunableNumber linearkI =
      new LoggedTunableNumber("AutonAlign/drivekI", 7.5);

  private static final LoggedTunableNumber thetakP =
      new LoggedTunableNumber("AutonAlign/thetakP", 4.5);
  private static final LoggedTunableNumber thetakD =
      new LoggedTunableNumber("AutonAlign/thetakD", 0.5);
  private static final LoggedTunableNumber linearTolerance =
      new LoggedTunableNumber(
          "AutonAlign/controllerLinearTolerance",
          Superstructure.getDesiredState() == SuperState.L4 ? 0.009 : 0.01);
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("AutonAlign/controllerThetaTolerance", Units.degreesToRadians(2));
  private static final LoggedTunableNumber toleranceTime =
      new LoggedTunableNumber("AutonAlign/controllerToleranceSecs", 0.25);
  //   private static final LoggedTunableNumber maxLinearVelocity =
  //       new LoggedTunableNumber(
  //           "AutonAlign/maxLinearVelocity", TunerConstants.driveConfig.maxLinearVelocity());
  //   private static final LoggedTunableNumber maxLinearAcceleration =
  //       new LoggedTunableNumber(
  //           "AutonAlign/maxLinearAcceleration",
  // TunerConstants.driveConfig.maxLinearAcceleration());
  private static final LoggedTunableNumber maxAngularVelocity =
      new LoggedTunableNumber(
          "AutonAlign/maxAngularVelocity", TunerConstants.driveConfig.maxAngularVelocity());
  private static final LoggedTunableNumber maxAngularAcceleration =
      new LoggedTunableNumber(
          "AutonAlign/maxAngularAcceleration",
          TunerConstants.driveConfig.maxAngularAcceleration() * 1);
  //   private static final LoggedTunableNumber slowLinearVelocity =
  //       new LoggedTunableNumber("AutonAlign/slowLinearVelocity",
  //       TunerConstants.driveConfig.maxLinearVelocity() * .4);
  //   private static final LoggedTunableNumber slowLinearAcceleration =
  //       new LoggedTunableNumber("AutonAlign/slowLinearAcceleration",
  //       TunerConstants.driveConfig.maxLinearAcceleration() * .7);
  //   private static final LoggedTunableNumber slowAngularVelocity =
  //       new LoggedTunableNumber("AutonAlign/slowAngularVelocity",
  //       TunerConstants.driveConfig.maxAngularVelocity() * 0.4);
  //   private static final LoggedTunableNumber slowAngularAcceleration =
  //       new LoggedTunableNumber("AutonAlign/slowAngularAcceleration",
  //       TunerConstants.driveConfig.maxAngularAcceleration() * 0.8);
  private static final LoggedTunableNumber ffMinRadius =
      new LoggedTunableNumber("AutonAlign/ffMinRadius", 0.4);
  private static final LoggedTunableNumber ffMaxRadius =
      new LoggedTunableNumber("AutonAlign/ffMaxRadius", 0.8);

  private final Pose2d desiredPose;
  private final Drive drive;
  //   private final Supplier<Translation2d> feedforwardSupplier;
  private final BooleanSupplier slowMode;
  private Translation2d lastSetpointTranslation;

  // Controllers for translation and rotation
  //   private PIDController linearPID;
  private ProfiledPIDController linearController;
  //   private TrapezoidProfile.State linearSetpoint;

  //   private PIDController thetaPID;
  private ProfiledPIDController thetaController;
  //   private TrapezoidProfile.State thetaSetpoint;

  private final Timer toleranceTimer = new Timer();

  public AutonReefAlignController(
      Drive drive,
      //   Supplier<Translation2d> feedforwardSupplier,
      BooleanSupplier slowMode,
      Pose2d desiredpose) {
    this.drive = drive;
    this.desiredPose = desiredpose;

    // this.feedforwardSupplier = feedforwardSupplier;
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
    }
    if (drive.getPose().getTranslation().getDistance(desiredPose.getTranslation()) > .1) {

      linearController.setI(linearkI.get());
    }

    if (slowMode.getAsBoolean()) {
      //   linearController.setConstraints(
      //       new TrapezoidProfile.Constraints(slowLinearVelocity.get(),
      // slowLinearAcceleration.get()));
      //   thetaController.setConstraints(
      //       new TrapezoidProfile.Constraints(
      //           slowAngularVelocity.get(), slowAngularAcceleration.get()));
      //   linearController.setPID(linearkP.get() * 2.2, linearkI.get(), linearkD.get() * 2.1);

      linearController.setP(linearkP.get() * 2.2); // to be tuned

      linearController.setD(linearkD.get() * 2.1);
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
    double linearVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond)
            .getNorm();
    // This one works
    // linearController.reset(
    //     currentPose.getTranslation().getDistance(goalPose.getTranslation()), linearVelocity);

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
            (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
            0.0,
            1.0);

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

    Logger.recordOutput("AutonAlign/MaxVel", linearController.getConstraints().maxVelocity);
    Logger.recordOutput("AutonAlign/MaxAccel", linearController.getConstraints().maxAcceleration);

    Logger.recordOutput("AutonAlign/DistanceMeasured", currentDistance);
    Logger.recordOutput("AutonAlign/DistanceSetpoint", linearController.getSetpoint().position);

    Logger.recordOutput("AutonAlign/VelocitySetpoint", driveVelocityScalar);
    Logger.recordOutput(
        "AutonAlign/'calculate'velocitySetPoint", linearController.calculate(currentDistance, 0.0));

    Logger.recordOutput("AutonAlign/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("AutonAlign/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "AutonAlign/SetpointPose",
        new Pose2d(
            lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
    Logger.recordOutput("Odometry/GoalPose", targetPose);
    // Logger.recordOutput("AutonAlign/AtGoal", atGoal());
    Logger.recordOutput(
        "AutonAlign/AtGoalCondition", (!linearController.atGoal() || !thetaController.atGoal()));
    Logger.recordOutput("AutonAlign/ToleranceTimer", toleranceTimer.get());

    // Command speeds
    var driveVelocity =
        new Pose2d(
                new Translation2d(0, 0),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation();
    // .plus(feedforwardSupplier.get());
    final double finalThetaVelocity = thetaVelocity;

    updateConstraints();
    return () ->
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(),
            driveVelocity.getY(),
            finalThetaVelocity,
            currentPose.getRotation());
  }

  @AutoLogOutput(key = "AutonAlign/AtGoal")
  public boolean atGoal() {
    // Logger.recordOutput("AutonAlign/AtGoal", toleranceTimer.get() > toleranceTime.get());
    return toleranceTimer.get() > toleranceTime.get();
  }
}

// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.Vector;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.*;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.numbers.N2;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.RobotState;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.util.GeomUtil;
// import frc.robot.util.LoggedTunableNumber;
// import java.util.function.BooleanSupplier;
// import java.util.function.Supplier;
// import lombok.experimental.ExtensionMethod;
// import org.littletonrobotics.junction.AutoLogOutput;
// import org.littletonrobotics.junction.Logger;

// @ExtensionMethod({GeomUtil.class})
// public class ReefAlignController {
//   private static final LoggedTunableNumber linearkP =
//       new LoggedTunableNumber("AutonAlign/drivekP", .8);
//   private static final LoggedTunableNumber linearkD =
//       new LoggedTunableNumber("AutonAlign/drivekD", 0.0);

//   private static final LoggedTunableNumber linearkI =
//       new LoggedTunableNumber("AutonAlign/drivekI", 20);

//   private static final LoggedTunableNumber thetakP =
//       new LoggedTunableNumber("AutonAlign/thetakP", 2.5);
//   private static final LoggedTunableNumber thetakD =
//       new LoggedTunableNumber("AutonAlign/thetakD", 0.5);
//   private static final LoggedTunableNumber linearTolerance =
//       new LoggedTunableNumber("AutonAlign/controllerLinearTolerance", 0.008);
//   private static final LoggedTunableNumber thetaTolerance =
//       new LoggedTunableNumber("AutonAlign/controllerThetaTolerance", Units.degreesToRadians(3));
//   private static final LoggedTunableNumber toleranceTime =
//       new LoggedTunableNumber("AutonAlign/controllerToleranceSecs", 0.1);
//   private static double maxLinearVelocity =
//       RobotState.getInstance().getModuleLimits().maxDriveVelocity() * .8;
//   //   private static final LoggedTunableNumber maxLinearVelocity =
//   //       new LoggedTunableNumber(
//   //           "AutonAlign/maxLinearVelocity",
//   //           RobotState.getInstance().getModuleLimits().maxDriveVelocity() * .5);
//   private static double maxLinearAcceleration =
//       RobotState.getInstance().getModuleLimits().maxDriveAcceleration() * 0.8;

//   //   private static final LoggedTunableNumber maxLinearAcceleration =
//   //       new LoggedTunableNumber(
//   //           "AutonAlign/maxLinearAcceleration",
//   //           RobotState.getInstance().getModuleLimits().maxDriveAcceleration() * 0.8);
//   private static final LoggedTunableNumber maxAngularVelocity =
//       new LoggedTunableNumber(
//           "AutonAlign/maxAngularVelocity", TunerConstants.driveConfig.maxAngularVelocity() *
// 0.6);
//   private static final LoggedTunableNumber maxAngularAcceleration =
//       new LoggedTunableNumber(
//           "AutonAlign/maxAngularAcceleration",
//           TunerConstants.driveConfig.maxAngularAcceleration() * 0.6);
//   //   private static final LoggedTunableNumber slowLinearVelocity =
//   //       new LoggedTunableNumber("AutonAlign/slowLinearVelocity",
//   //       TunerConstants.driveConfig.maxLinearVelocity() * .4);
//   //   private static final LoggedTunableNumber slowLinearAcceleration =
//   //       new LoggedTunableNumber("AutonAlign/slowLinearAcceleration",
//   //       TunerConstants.driveConfig.maxLinearAcceleration() * .7);
//   //   private static final LoggedTunableNumber slowAngularVelocity =
//   //       new LoggedTunableNumber("AutonAlign/slowAngularVelocity",
//   //       TunerConstants.driveConfig.maxAngularVelocity() * 0.4);
//   //   private static final LoggedTunableNumber slowAngularAcceleration =
//   //       new LoggedTunableNumber("AutonAlign/slowAngularAcceleration",
//   //       TunerConstants.driveConfig.maxAngularAcceleration() * 0.8);
//   private static final LoggedTunableNumber ffMinRadius =
//       new LoggedTunableNumber("AutonAlign/ffMinRadius", 0.2);
//   private static final LoggedTunableNumber ffMaxRadius =
//       new LoggedTunableNumber("AutonAlign/ffMaxRadius", 0.8);

//   private final Pose2d desiredPose;
//   private final Drive drive;
//   //   private final Supplier<Translation2d> feedforwardSupplier;
//   private final BooleanSupplier slowMode;
//   private final BooleanSupplier toggle;
//   private Translation2d lastSetpointTranslation;

//   // Controllers for translation and rotation
//   //   private PIDController linearPID;
//   private ProfiledPIDController linearController;
//   //   private TrapezoidProfile.State linearSetpoint;

//   //   private PIDController thetaPID;
//   private ProfiledPIDController thetaController;
//   //   private TrapezoidProfile.State thetaSetpoint;

//   private final Timer toleranceTimer = new Timer();

//   public ReefAlignController(
//       Drive drive,
//       //   Supplier<Translation2d> feedforwardSupplier,
//       BooleanSupplier slowMode,
//       BooleanSupplier toggle) {
//     this.drive = drive;
//     this.toggle = toggle;
//     this.desiredPose = RobotState.getInstance().getNearestReefPose(drive.getPose(), toggle);

//     // this.feedforwardSupplier = feedforwardSupplier;
//     this.slowMode = slowMode;
//     // Set up both controllers

//     linearController =
//         new ProfiledPIDController(
//             linearkP.get(), 0, linearkD.get(), new TrapezoidProfile.Constraints(0, 0));
//     linearController.setTolerance(linearTolerance.get());
//     thetaController =
//         new ProfiledPIDController(
//             thetakP.get(), 0, thetakD.get(), new TrapezoidProfile.Constraints(0, 0));
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);
//     thetaController.setTolerance(thetaTolerance.get());
//     toleranceTimer.restart();
//     updateConstraints();
//     resetControllers();
//   }

//   private void updateConstraints() {

//     maxLinearVelocity = RobotState.getInstance().getModuleLimits().maxDriveVelocity() * .8;

//     maxLinearAcceleration = RobotState.getInstance().getModuleLimits().maxDriveAcceleration() *
// 0.8;

//     if (drive.getPose().getTranslation().getDistance(desiredPose.getTranslation()) < .1) {

//       linearController.setI(linearkI.get() * 15);
//     }
//     if (drive.getPose().getTranslation().getDistance(desiredPose.getTranslation()) > .1) {

//       linearController.setI(linearkI.get());
//     }

//     if (slowMode.getAsBoolean()) {
//       linearController.setConstraints(
//           new TrapezoidProfile.Constraints(maxLinearVelocity, maxLinearAcceleration));
//       thetaController.setConstraints(
//           new TrapezoidProfile.Constraints(maxAngularVelocity.get(),
// maxAngularAcceleration.get()));
//       linearController.setP(linearkP.get() * 2.2);
//       linearController.setD(linearkD.get() * 2.1);
//       //   linearController.setIZone(0.2);

//     } else {
//       linearController.setConstraints(
//           new TrapezoidProfile.Constraints(maxLinearVelocity, maxLinearAcceleration));
//       thetaController.setConstraints(
//           new TrapezoidProfile.Constraints(maxAngularVelocity.get(),
// maxAngularAcceleration.get()));
//       linearController.setPID(linearkP.get(), linearkI.get(), linearkD.get());
//       linearController.setIZone(0.2);
//     }
//   }

//   private void resetControllers() {
//     // Reset measurements and velocities
//     Pose2d currentPose = drive.getPose();
//     Pose2d goalPose = desiredPose;
//     Twist2d fieldVelocity = GeomUtil.toTwist2d(drive.getChassisSpeeds());
//     // Rotation2d robotToGoalAngle =
//     //     goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();

//     // The following few lines of code are to reset the controller based on the current position
// and
//     // the current velocity.
//     // The vector stuff is to find the magnitude of the velocity going towards the setpoint
// already
//     // where negative direction is towards the setpoint. (negated in the reset line at the end)

//     double linearVelocity = new Translation2d(fieldVelocity.dx, fieldVelocity.dy).getNorm();
//     Vector<N2> currentPoseToDesiredPoseVector =
//         desiredPose.minus(drive.getPose()).getTranslation().toVector();
//     Vector<N2> currentVelocityVector =
//         new Translation2d(fieldVelocity.dx, fieldVelocity.dy).toVector();
//     // Translation2d currentVelocityTranslation = new Translation2d(fieldVelocity.dx,
//     // fieldVelocity.dy);

//     double poseToDesiredPoseCosAngle =
//         currentPoseToDesiredPoseVector.dot(currentVelocityVector)
//             / currentPoseToDesiredPoseVector.norm()
//             / currentVelocityVector.norm();
//     // double poseToDesiredPoseActualAngle = Math.acos(poseToDesiredPoseCosAngle); turns out we
//     // didn't need it
//     linearController.reset(
//         currentPose.getTranslation().getDistance(goalPose.getTranslation()),
//         linearVelocity
//             * /* negated because positive is away from the setpoint, negative is towards the
// setpoint*/ -poseToDesiredPoseCosAngle);

//     thetaController.reset(currentPose.getRotation().getRadians());
//     lastSetpointTranslation = currentPose.getTranslation();
//   }

//   public Supplier<ChassisSpeeds> update() {
//     // Update Controllers
//     LoggedTunableNumber.ifChanged(
//         hashCode(),
//         () -> linearController.setPID(linearkP.get(), 0, linearkD.get()),
//         linearkP,
//         linearkD);
//     LoggedTunableNumber.ifChanged(
//         hashCode(),
//         () -> thetaController.setPID(thetakP.get(), 0, thetakD.get()),
//         thetakP,
//         thetakD);
//     LoggedTunableNumber.ifChanged(
//         hashCode(), () -> linearController.setTolerance(linearTolerance.get()), linearTolerance);
//     LoggedTunableNumber.ifChanged(
//         hashCode(), () -> thetaController.setTolerance(thetaTolerance.get()), thetaTolerance);
//     // LoggedTunableNumber.ifChanged(
//     //     hashCode(),
//     //     this::updateConstraints,
//     //     maxLinearVelocity,
//     //     maxLinearAcceleration,
//     //     // slowLinearVelocity,
//     //     // slowLinearAcceleration,
//     //     maxAngularVelocity,
//     //     maxAngularAcceleration
//     //     // slowAngularVelocity,
//     //     // slowAngularAcceleration
//     //     );

//     // Control to setpoint
//     Pose2d currentPose = drive.getPose();
//     Pose2d targetPose = desiredPose;

//     // Calculate drive speed
//     double currentDistance =
// currentPose.getTranslation().getDistance(targetPose.getTranslation());
//     double ffScaler = // 1.0;
//         MathUtil.clamp(
//             (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
//             0.0,
//             1.0);

//     linearController.reset(
//         linearController.getSetpoint().position, linearController.getSetpoint().velocity);
//     // thetaController.reset(thetaController.getSetpoint().position,
//     // thetaController.getSetpoint().velocity);

//     double driveVelocityScalar =
//         linearController.getSetpoint().velocity * ffScaler
//             + linearController.calculate(currentDistance, 0.0);

//     if (linearController.atGoal()) driveVelocityScalar = 0.0;
//     lastSetpointTranslation =
//         new Pose2d(
//                 targetPose.getTranslation(),
//                 currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
//             .transformBy(GeomUtil.toTransform2d(linearController.getSetpoint().position, 0.0))
//             .getTranslation();

//     // Calculate theta speed
//     double thetaVelocity =
//         // thetaController.getSetpoint().velocity * ffScaler
//         thetaController.calculate(
//             currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
//     if (thetaController.atGoal()) thetaVelocity = 0.0;

//     // Reset tolerance timer
//     if (!linearController.atGoal() || !thetaController.atGoal()) {
//       toleranceTimer.reset();
//     }

//     // Log data

//     Logger.recordOutput("AutonAlign/MaxVel", linearController.getConstraints().maxVelocity);
//     Logger.recordOutput("AutonAlign/MaxAccel",
// linearController.getConstraints().maxAcceleration);

//     Logger.recordOutput("AutonAlign/DistanceMeasured", currentDistance);
//     Logger.recordOutput("AutonAlign/DistanceSetpoint", linearController.getSetpoint().position);

//     Logger.recordOutput("AutonAlign/VelocitySetpoint", driveVelocityScalar);
//     Logger.recordOutput(
//         "AutonAlign/'calculate'velocitySetPoint", linearController.calculate(currentDistance,
// 0.0));

//     Logger.recordOutput("AutonAlign/ThetaMeasured", currentPose.getRotation().getRadians());
//     Logger.recordOutput("AutonAlign/ThetaSetpoint", thetaController.getSetpoint().position);
//     Logger.recordOutput(
//         "AutonAlign/SetpointPose",
//         new Pose2d(
//             lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
//     Logger.recordOutput("Odometry/GoalPose", targetPose);
//     // Logger.recordOutput("AutonAlign/AtGoal", atGoal());
//     Logger.recordOutput(
//         "AutonAlign/AtGoalCondition", (!linearController.atGoal() || !thetaController.atGoal()));
//     Logger.recordOutput("AutonAlign/ToleranceTimer", toleranceTimer.get());

//     // Command speeds
//     var driveVelocity =
//         new Pose2d(
//                 new Translation2d(0, 0),
//                 currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
//             .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
//             .getTranslation();
//     // .plus(feedforwardSupplier.get());
//     final double finalThetaVelocity = thetaVelocity;

//     updateConstraints();
//     return () ->
//         ChassisSpeeds.fromFieldRelativeSpeeds(
//             driveVelocity.getX(),
//             driveVelocity.getY(),
//             finalThetaVelocity,
//             currentPose.getRotation());
//   }

//   @AutoLogOutput(key = "AutonAlign/AtGoal")
//   public boolean atGoal() {
//     // Logger.recordOutput("AutonAlign/AtGoal", toleranceTimer.get() > toleranceTime.get());
//     return toleranceTimer.get() > toleranceTime.get();
//   }
// }
