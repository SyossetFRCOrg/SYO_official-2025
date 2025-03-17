package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 4.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY =
      TunerConstants.driveConfig.maxAngularVelocity() * 1.5;
  private static final double ANGLE_MAX_ACCELERATION =
      TunerConstants.driveConfig.maxAngularAcceleration() * 1.5;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  // private static Pose2d[]
  //     reefscoringPositions = // not finalized or tuned. pose2d of all the blue reef scoring
  // // positions
  // { // use alliancefliputil to flip to get corresponding red scoring pose2ds
  //   new Pose2d(3.2292869091033936, 3.8667519092559814, Rotation2d.fromDegrees(0)),
  //   new Pose2d(3.2235898971557617, 4.180093288421631, Rotation2d.fromDegrees(0)),
  //   new Pose2d(3.707775115966797, 5.033270359039307,
  // Rotation2d.fromRadians(-1.0466175637493382)),
  //   new Pose2d(3.985335350036621, 5.185656547546387,
  // Rotation2d.fromRadians(-1.0466175637493382)),
  //   new Pose2d(4.970402717590332, 5.174771785736084,
  // Rotation2d.fromRadians(-2.0988710476023327)),
  //   new Pose2d(5.264289855957031, 5.011500835418701,
  // Rotation2d.fromRadians(-2.0988710476023327)),
  //   new Pose2d(5.726890563964844, 4.184262275695801, Rotation2d.fromDegrees(180)),
  //   new Pose2d(5.732332706451416, 3.868605375289917, Rotation2d.fromDegrees(180)),
  // };

  private DriveCommands() {}

  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX()
                      * RobotState.getInstance().getModuleLimits().maxDriveVelocity(),
                  linearVelocity.getY()
                      * RobotState.getInstance().getModuleLimits().maxDriveVelocity(),
                  omega * TunerConstants.driveConfig.maxAngularVelocity());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Uses joystickDriveAtAngle, just provides a the Rotation2d supplier depending on the current
   * estimated pose, looks at the nearest coral station.
   *
   * <p>joystickDriveCoralStation already flips the pose by PI radians if it's red alliance.
   *
   * @return
   */
  public static Command joystickDriveCoralStation(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    // // Logger.recordOutput("Odometry/botPoseY>4", 4.0 > drive.getPose().getY());
    // if (4.0 > yCoordinateSupplier.getAsDouble()) //doesn't work right now
    // {
    //   //blue left side coral station

    // extremely jank solution

    return DriveCommands.joystickDriveAtAngle(
        drive,
        xSupplier,
        ySupplier,
        () -> RobotState.getInstance().getNearestCoralStationPose(drive.getPose()).getRotation());
    // }
    // else if (4.0 <= yCoordinateSupplier.getAsDouble())
    // { //blue right side coral station
    // return DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, () ->
    // Rotation2d.fromRadians(-2.2025756098633624)).onlyIf(() -> 4.0 <=
    // yCoordinateSupplier.getAsDouble());
    // }
    // else{
    // return Commands.none();
    // }
    // return DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, drive.getPose().getY()
    // > 4.0 ? () -> Rotation2d.fromRadians(2.2020204805272137) : () ->
    // Rotation2d.fromRadians(-2.2020204805272137));

  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(Units.degreesToRadians(2));

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * TunerConstants.driveConfig.maxLinearVelocity(),
                      linearVelocity.getY() * TunerConstants.driveConfig.maxLinearVelocity(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  // public static double getDistanceToNearestReef(Pose2d pose) {
  //   double mindistance = Double.POSITIVE_INFINITY;
  //   int index = -1;
  //   for (int i = 0; i < reefscoringPositions.length; i++) {
  //     if (pose.getTranslation().getDistance(reefscoringPositions[i].getTranslation())
  //         < mindistance) {
  //       index = i;
  //       mindistance =
  // pose.getTranslation().getDistance(reefscoringPositions[i].getTranslation());
  //     }
  //   }

  //   return mindistance;
  // }

  // public static Pose2d getNearestReefPose(Pose2d pose) {
  //   double mindistance = Double.POSITIVE_INFINITY;
  //   int index = -1;
  //   for (int i = 0; i < reefscoringPositions.length; i++) {
  //     if (pose.getTranslation().getDistance(reefscoringPositions[i].getTranslation())
  //         < mindistance) {
  //       index = i;
  //       mindistance =
  // pose.getTranslation().getDistance(reefscoringPositions[i].getTranslation());
  //     }
  //   }

  //   return reefscoringPositions[index];
  // }

  // public static PathPlannerPath getPathToNearestReef(Pose2d pose) {

  //   double mindistance = Double.POSITIVE_INFINITY;
  //   int index = -1;
  //   for (int i = 0; i < reefscoringPositions.length; i++) {
  //     if (pose.getTranslation().getDistance(reefscoringPositions[i].getTranslation())
  //         < mindistance) {
  //       index = i;
  //       mindistance =
  // pose.getTranslation().getDistance(reefscoringPositions[i].getTranslation());
  //     }
  //   }

  //   /**
  //    * The waypointsFromPoses method required that the rotation component of each pose is the
  //    * direction of travel, not the rotation of a swerve chassis.
  //    *
  //    * <p>To set the rotation the path should end with, use the GoalEndState.
  //    */

  //   // if this works i'm gonna go crazy
  //   List<Waypoint> waypoints =
  //       PathPlannerPath.waypointsFromPoses(
  //           new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(0)),
  //           new Pose2d(
  //               pose.getTranslation()
  //                   .interpolate(reefscoringPositions[index].getTranslation(), 0.5),
  //               reefscoringPositions[index]
  //                   .getTranslation()
  //                   .minus(pose.getTranslation())
  //                   .getAngle()),
  //           reefscoringPositions[index]);

  //   // List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
  //   //   new Pose2d(0,4,Rotation2d.fromDegrees(0)),
  //   //   new Pose2d(2,4,Rotation2d.fromDegrees(0))
  //   //   //,
  //   //   // new Pose2d(4,4,Rotation2d.fromDegrees(0))
  //   //   );

  //   PathConstraints constraints =
  //       new PathConstraints(
  //           TunerConstants.moduleLimitsFree.maxDriveVelocity() * .8,
  //           TunerConstants.moduleLimitsFree.maxDriveAcceleration() * .8,
  //           TunerConstants.moduleLimitsFree.maxSteeringVelocity() * .8,
  //           TunerConstants.moduleLimitsFree.maxSteeringVelocity()
  //               * 1.5); // The constraints for this path.
  //   // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also
  // use
  //   // unlimited constraints, only limited by motor torque and nominal battery voltage

  //   // Create the path using the waypoints created above
  //   PathPlannerPath path =
  //       new PathPlannerPath(
  //           waypoints,
  //           constraints,
  //           null, // The ideal starting state, this is only relevant for pre-planned paths, so
  // can
  //           // be null for on-the-fly paths.
  //           new GoalEndState(
  //               0.0,
  //               reefscoringPositions[index]
  //                   .getRotation()) // Goal end state. You can set a holonomic rotation here. If
  //           // using a differential drivetrain, the rotation will have no
  //           // effect.
  //           );

  //   // Prevent the path from being flipped if the coordinates are already correct
  //   path.preventFlipping = true;

  //   return path;
  // }

  // public static Command lineUpToNearestReef(Supplier<Pose2d> pose) {
  //   // have to seperate making the path and running it because making the path is not a
  // "runnable"
  //   return (AutoBuilder.followPath(getPathToNearestReef(pose.get())));
  // }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
