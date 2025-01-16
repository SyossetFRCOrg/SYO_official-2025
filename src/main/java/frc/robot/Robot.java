// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.DriveModuleSpark;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.GenericSwerveModule;
import frc.robot.subsystems.drive.SonicSwerveDrivetrain;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.TurnModuleSpark;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final Drivetrain drivetrain;
  private final CommandXboxController controller;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    drivetrain = createDrivetrain();

    controller = new CommandXboxController(0);
  }

  private Drivetrain createDrivetrain() {
    Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
    );

    DriveModuleSpark.Constants.Builder driveBuilder = new DriveModuleSpark.Constants.Builder()
      .setVolts(12)
      .setWheelRadiusMeters(Units.inchesToMeters(1.7));

    SparkMax driveSparks[] = {
      new SparkMax(1, MotorType.kBrushless),
      new SparkMax(4, MotorType.kBrushless),
      new SparkMax(7, MotorType.kBrushless),
      new SparkMax(10, MotorType.kBrushless),
    };
    
    SparkMax turnSparks[] = {
      new SparkMax(2, MotorType.kBrushless),
      new SparkMax(5, MotorType.kBrushless),
      new SparkMax(8, MotorType.kBrushless),
      new SparkMax(11, MotorType.kBrushless),
    };

    CANcoder cancoders[] = {
      new CANcoder(3, "rio"),
      new CANcoder(6, "rio"),
      new CANcoder(9, "rio"),
      new CANcoder(12, "rio")
    };

    for (var cancoder : cancoders) {      
      cancoder.getConfigurator().apply(new CANcoderConfiguration());
    }
    
    var driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20, 20)
        .voltageCompensation(12.0);
    
    for (var drive : driveSparks) {
      drive.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20)
        .voltageCompensation(12.0);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / 50.0))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    
    for (var turn : turnSparks) {
      turn.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    var driveConstants = new DriveModuleSpark.Constants.Builder()
      .setVolts(12)
      .setWheelRadiusMeters(Units.inchesToMeters(1.7));
    
    var turnConstants = new TurnModuleSpark.Constants.Builder()
      .setTurnMotorReduction(150.0 / 7.0)
      .setPID(5.0, 0.0, 0.0);
    
    return new SonicSwerveDrivetrain(kinematics, new SwerveModule[] {
      new GenericSwerveModule(new DriveModuleSpark(driveSparks[0], driveConstants.build()), new TurnModuleSpark(turnSparks[0], cancoders[0], 
        turnConstants.setZeroRotation(new Rotation2d(Math.PI/2)).build())),
      new GenericSwerveModule(new DriveModuleSpark(driveSparks[1], driveConstants.build()), new TurnModuleSpark(turnSparks[1], cancoders[1], 
        turnConstants.setZeroRotation(new Rotation2d(-Math.PI/2)).build())),
      new GenericSwerveModule(new DriveModuleSpark(driveSparks[2], driveConstants.build()), new TurnModuleSpark(turnSparks[2], cancoders[2], 
        turnConstants.setZeroRotation(new Rotation2d(0)).build())),
      new GenericSwerveModule(new DriveModuleSpark(driveSparks[3], driveConstants.build()), new TurnModuleSpark(turnSparks[3], cancoders[3], 
        turnConstants.setZeroRotation(new Rotation2d(0)).build()))
    });
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    drivetrain.setSpeeds(new ChassisSpeeds(controller.getLeftY() * 2, controller.getLeftX() * 2, controller.getRightX() * 4));
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
