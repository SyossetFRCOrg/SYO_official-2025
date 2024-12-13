// Copyright 2021-2024 FRC 6328
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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;
import frc.robot.util.LocalADStarAK;;

public class Drive extends SubsystemBase {      
  private static final double MAX_LINEAR_SPEED = (5600.0 / 60.0) / ModuleIOTalonFX.DRIVE_GEAR_RATIO * Units.inchesToMeters(1.95) * 2 * Math.PI;
  private static final double TRACK_WIDTH_X = .6;
  private static final double TRACK_WIDTH_Y = .6;
  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;

  private boolean enableRotationLock = false;


  private final PIDController snapToAnglePID = new PIDController(4.0, 0, .5);

  private double shootRotationTolerance = 0.07;
  private double passRotationTolerance = 0.07;

  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private ChassisSpeeds discreteSpeeds;

  private final GenericEntry m_distanceToNearestSpeakerEntry;
  // private final GenericEntry m_outtakeAngleEntry;
  NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("Drive");


  StructArrayPublisher<SwerveModuleState> desiredSwervePublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("desiredSwervePublisher", SwerveModuleState.struct).publish();

  StructArrayPublisher<SwerveModuleState> actualSwervePublisher = NetworkTableInstance.getDefault()
  .getStructArrayTopic("actualSwervePublisher", SwerveModuleState.struct).publish();

  StructPublisher<Pose2d> posepublisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose2d.struct).publish();



  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();



  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d(),
      VecBuilder.fill(Units.inchesToMeters(2.0), Units.inchesToMeters(2.0), Units.degreesToRadians(2.0)),
      VecBuilder.fill(Units.inchesToMeters(1.0), Units.inchesToMeters(1.0), Units.degreesToRadians(2.0)));
  
    


  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Start threads (no-op for each if no signals have been created)
    PhoenixOdometryThread.getInstance().start();
    // SparkMaxOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0),
              
            MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
        
            () -> Constants.getAlliance() == Alliance.Blue,
        this)
        
        ;
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });


    snapToAnglePID.enableContinuousInput(0.0, 2 * Math.PI);


    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));

      networkTable = NetworkTableInstance.getDefault().getTable("Drive");
      
      ShuffleboardTab tab = Shuffleboard.getTab("Drive");
      ShuffleboardLayout ShootingInfoLayout = tab.getLayout("ShootingInfoLayout", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0);
      
      m_distanceToNearestSpeakerEntry = ShootingInfoLayout.add("(m) Distance to Nearest Speaker", getDistanceToNearestSpeaker()).getEntry();
      // m_outtakeAngleEntry = ShootingInfoLayout.add("Desired Outtake Angle", calculateOuttakeAngle() + " rad").getEntry();
        
  }

  public void periodic() {

    m_distanceToNearestSpeakerEntry.setDouble(getDistanceToNearestSpeaker());
    // m_outtakeAngleEntry.setString(calculateOuttakeAngle() + " rad");


    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();

    posepublisher.set(getPose());
    }

    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

    // Whether, or not, to compensate for translational skew when translating and rotating
    // DON'T USE Timer.getFPGATimestamp() here. We occasionally get the same value that we've already stored.
    // If dt == 0 ChassisSpeedHelper.discretize() produces invalid values (infinity or NaN).
    // Converts microseconds from HALUtil to seconds
    // double timeStamp = HALUtil.getFPGATime() / 1.0e6;
    
    if (enableRotationLock) {

      if (!atSetpoint()){ //if not at the setpoint, then you continue to aim.
            double rotationalVelocity = MathUtil.clamp(
                    snapToAnglePID.calculate(
                                  getPose()
                                    .getRotation()
                                    .getRadians() + Units.degreesToRadians(180),
                            getSnaptoAnglePID().getRadians()),
                    -0.6 * MAX_ANGULAR_SPEED,
                    0.6 * MAX_ANGULAR_SPEED);
                    var fieldRel = ChassisSpeeds.fromRobotRelativeSpeeds(
                    discreteSpeeds,
                    getPose().getRotation());
            fieldRel.omegaRadiansPerSecond = rotationalVelocity;
            discreteSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    fieldRel, getPose().getRotation());
            }
      else{//if at the setpoint, just stop rotating, so can actually shoot and stop visiting.
        double rotationalVelocity = 0;
        var fieldRel = ChassisSpeeds.fromRobotRelativeSpeeds(
                    discreteSpeeds,
                    getPose().getRotation());
            fieldRel.omegaRadiansPerSecond = rotationalVelocity;
            discreteSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    fieldRel, getPose().getRotation());
      }
    }
      

                    

    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, discreteSpeeds, MAX_LINEAR_SPEED, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }


    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    desiredSwervePublisher.set(optimizedSetpointStates);
    actualSwervePublisher.set(getModuleStates());
    
  }


  public double getVelocity(){
    return Math.sqrt(Math.pow(discreteSpeeds.vxMetersPerSecond,2) + Math.pow(discreteSpeeds.vyMetersPerSecond,2));
  }

  public boolean stopped(){
    return MathUtil.isNear(getVelocity(), 0, .03);
  }
  

  public ChassisSpeeds getChassisSpeeds(){
    return kinematics.toChassisSpeeds(getModuleStates());
  }
 


  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public Command setRotationLock() {
    return new InstantCommand(() -> enableRotationLock = true );
    
}

public Command disableRotationLock() {
    return new InstantCommand(() -> enableRotationLock = false );
}

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public double getDriveBaseRadius(){
    return DRIVE_BASE_RADIUS;
  }


  /** Returns the current rotation tolerance. */
  public double getShootRotationTolerance() {
    return shootRotationTolerance;
  }

  /** Returns the current rotation tolerance. */
  public double getPassRotationTolerance() {
    return passRotationTolerance;
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  
  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }

  /**
     * guesses distance to nearest speaker using the swerveDrivePoseEstimator
     * 
     * 
     */
    private double getDistanceToNearestSpeaker() {

      Pose2d blue_speaker = new Pose2d(-0.0381, 5.5479, new Rotation2d());

      Pose2d red_speaker = new Pose2d(16.579, 5.5479, new Rotation2d());
      
      double distance_blue = Math.sqrt(Math.pow(getPose().getX() - blue_speaker.getX(),2) + Math.pow(getPose().getY() - blue_speaker.getY(),2));
      double distance_red = Math.sqrt(Math.pow(getPose().getX() - red_speaker.getX() ,2) + Math.pow(getPose().getY() - red_speaker.getY(),2));


      
    switch (Constants.getAlliance()) {
      case Red:
          return distance_red;
      case Blue:
          return distance_blue;
      default:
        return 1.3;
    }
  }

    /**
     * guesses angle to nearest speaker using the swerveDrivePoseEstimator
     * 
     * 
     * @param boolean LimelightShot: true if doing a shot, will return speaker's rotation2d. Else, will give it 5degrees more towards the ampside, for a pass
     * @return Rotation2d angle that snaptoPID will use
     * 
     */
    private Rotation2d getSnaptoAnglePID() {

      Pose2d blue_speaker = new Pose2d(-0.0381, 5.5479, new Rotation2d());

      Pose2d red_speaker = new Pose2d(16.579, 5.5479, new Rotation2d());

      switch (Constants.getAlliance()) {
      case Red:
      
        switch (Superstructure.currentSuperState){
          case PREPARING_LIMELIGHT_SHOT:
          case LIMELIGHT_SHOT:
            return new Rotation2d(Units.degreesToRadians(180) - Math.atan((getPose().getY() - red_speaker.getY()) / (getPose().getX() - red_speaker.getX())));
            case PASS:
            case PREPARING_PASS:
            return new Rotation2d(Units.degreesToRadians(185) - Math.atan((getPose().getY() - red_speaker.getY()) / (getPose().getX() - red_speaker.getX())));
            
          default:
            return new Rotation2d();
      }
      case Blue:
        switch (Superstructure.currentSuperState){
          case LIMELIGHT_SHOT:
          case PREPARING_LIMELIGHT_SHOT:
            return new Rotation2d(Units.degreesToRadians(180) - Math.atan((getPose().getY() - blue_speaker.getY()) / (getPose().getX() - blue_speaker.getX())));
          case PASS:
          case PREPARING_PASS:
            return new Rotation2d(Units.degreesToRadians(175) - Math.atan((getPose().getY() - blue_speaker.getY()) / (getPose().getX() - blue_speaker.getX())));
          default:
          return new Rotation2d();
          
    }
      default:
      return new Rotation2d();
      
    }
  }
    
      // Rotation2d blue_theta = new Rotation2d(Units.degreesToRadians(180) - Math.atan((getPose().getY() - blue_speaker.getY()) / (getPose().getX() - blue_speaker.getX())));
      
      // Rotation2d red_theta = new Rotation2d(Units.degreesToRadians(180) - Math.atan((getPose().getY() - red_speaker.getY()) / (getPose().getX() - red_speaker.getX())));
    

      // return blue_theta;



    /**
   * Calculates the optimal outtake angle for shooting based on limelight trigonometric input.
   * 
   * @return Optimal outtake absolute angle (rad).
   */
  public double calculateShootAngle() {
      return 1.274 * Math.pow(.546087, getDistanceToNearestSpeaker()) - 3.665;
      // return 1.27341 * Math.pow(.546087, getDistanceToNearestSpeaker()) - 3.67942;
  }

  /**
   * Calculates the optimal outtake angle for passing based on limelight trigonometric input.
   * 
   * @return Optimal outtake absolute angle (rad).
   */
  public double calculatePassAngle() {
      return -3.08;
  }

    public boolean atSetpoint(){
      switch (Superstructure.currentSuperState){
        case LIMELIGHT_SHOT:
        case PREPARING_LIMELIGHT_SHOT:
            return MathUtil.isNear(getPose().getRotation().getRadians(), getSnaptoAnglePID().getRadians(), shootRotationTolerance);
        case PASS:
        case PREPARING_PASS:
          return MathUtil.isNear(getPose().getRotation().getRadians(), getSnaptoAnglePID().getRadians(), passRotationTolerance);
        default:
          return false;
      }
    }
    
    

}