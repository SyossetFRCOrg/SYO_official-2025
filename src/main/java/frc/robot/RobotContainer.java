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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Superstructure;
// import frc.robot.subsystems.climber.ClimberSubsystem;
// import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.vision.Vision;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Flywheel flywheel;
  private final Superstructure superstructure;
  // private final ClimberSubsystem climberSubsystem;
  private final Intake intake;
// 

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);

  private final UsbCamera m_camera;
  private final HttpCamera m_limelight;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public double speedMultiplier;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        
        drive = new Drive(
        new GyroIOPigeon2(true),
        new ModuleIOTalonFX(0),
        new ModuleIOTalonFX(1),
        new ModuleIOTalonFX(2),
        new ModuleIOTalonFX(3)
        );
        flywheel = new Flywheel(new FlywheelIOSparkMax(), drive);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        flywheel = new Flywheel(new FlywheelIOSim(), drive);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        flywheel = new Flywheel(new FlywheelIO() {}, drive);
        break;
    }
    // climberSubsystem = new ClimberSubsystem();

    intake = new Intake(new IntakeIOSparkMax() {});

    superstructure = new Superstructure(drive, flywheel, intake, this);
    

    speedMultiplier = 1;

    // // Set up auto routines
    // NamedCommands.registerCommand(
    //     "Run Flywheel",
    //     Commands.startEnd(
    //             () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
    //         .withTimeout(5.0));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Flywheel SysId (Quasistatic Forward)",
    //     flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Flywheel SysId (Quasistatic Reverse)",
    //     flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Flywheel SysId (Dynamic Forward)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Flywheel SysId (Dynamic Reverse)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings


    m_camera = CameraServer.startAutomaticCapture();
    m_camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    m_camera.setResolution(80, 60);
    Shuffleboard.getTab("Match").add(m_camera).withWidget(BuiltInWidgets.kCameraStream).withSize(4, 3).withPosition(4, 3);
    m_limelight = new HttpCamera("Limelight", "http://limelight.local:5800/stream.mjpeg");
    m_limelight.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    Shuffleboard.getTab("Match").add(m_limelight).withWidget(BuiltInWidgets.kCameraStream).withSize(2, 2).withPosition(4, 0);


    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * speedMultiplier,
            () -> -controller.getLeftX() * speedMultiplier,
            () -> -controller.getRightX() * speedMultiplier));
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));


    
    controller
        .button(8)
        .onTrue( 
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), //new Rotation2d())),
                             Constants.getAlliance() == Alliance.Blue ? new Rotation2d() : Rotation2d.fromDegrees(180))),
                    drive)
                .ignoringDisable(true));

    controller
      .rightBumper()
      .whileTrue(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.INTAKE_DOWN))
      .onFalse(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.INTAKE_UP));


    // fixing the note if it's wrongly placed.
    controller
        .rightTrigger()
        .whileTrue(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.MANUAL)
        .alongWith(new InstantCommand(() -> intake.intake(() -> 5.0)).repeatedly()))
        .whileFalse(Commands.deadline(waitSeconds(.25), superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE).repeatedly()).alongWith(drive.disableRotationLock()));
    controller
        .leftTrigger()
        .whileTrue(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.MANUAL)
        .alongWith(new InstantCommand(() -> intake.intake(() -> -5.0)).repeatedly()))
        .whileFalse(Commands.deadline(waitSeconds(.25), superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE).repeatedly()).alongWith(drive.disableRotationLock()));

    controller
        .b()        
        .onTrue(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.PASS))//.alongWith(drive.setRotationLock()))
        .whileFalse(Commands.deadline(waitSeconds(.25), superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE).repeatedly()).alongWith(drive.disableRotationLock()));

    controller
        .y()
        .onTrue(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.SUBWOOFER_SHOT))
        .whileFalse(Commands.deadline(waitSeconds(.25), superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE).repeatedly()).alongWith(drive.disableRotationLock()));

    controller.x().whileTrue(Commands.deadline(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE).repeatedly()).alongWith(drive.disableRotationLock()));
    
    
    // controller
    //     .a()
    //     .onTrue(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.LIMELIGHT_SHOT))//.alongWith(drive.setRotationLock()))
    //     .whileFalse(Commands.deadline(waitSeconds(.25), superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE).repeatedly()).alongWith(drive.disableRotationLock()));

    controller
        .povRight()
        .whileTrue(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.CLIMBER_UP).repeatedly())
        .whileFalse(Commands.deadline(waitSeconds(.25), superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE).repeatedly()).alongWith(drive.disableRotationLock()));
    
    controller
        .povLeft()
        .whileTrue(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.CLIMBER_DOWN).repeatedly())
        .whileFalse(Commands.deadline(waitSeconds(.25), superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE).repeatedly()).alongWith(drive.disableRotationLock()));
            

    // controller
    //     .povRight()
    //     .onTrue(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.HOLD_FIX_PIECE));
      
    controller
        .povUp()
        .onTrue(new InstantCommand(() -> speedMultiplier += .1 ).onlyIf(() -> speedMultiplier < 1));

    controller
        .povDown()
        .onTrue(new InstantCommand(() -> speedMultiplier -= .1 ).onlyIf(() -> speedMultiplier > 0));
        
    

    // controller2
    //   .povDown()
    //   .whileTrue(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.MANUAL)
    //     .alongWith(new InstantCommand(() -> intake.intake(() -> intake.getIntakeFeedForward().calculate(-400))).repeatedly()))
    //   .onFalse(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE));

    // controller2
    //   .povUp()
    //   .whileTrue(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.MANUAL)
    //     .alongWith(new InstantCommand(() -> intake.intake(() -> intake.getIntakeFeedForward().calculate(400))).repeatedly()))
    //   .onFalse(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE));

    controller
      .leftBumper()
      .whileTrue(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.MANUAL)
        .alongWith(new InstantCommand(() -> intake.rotate(() -> intake.getRotateFeedForward().calculate(0.5)))).repeatedly())
      .onFalse(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE)
      .andThen(() -> intake.reset()));



    controller
    .a()
    .whileTrue(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.MANUAL)
    .alongWith(new InstantCommand( () -> flywheel.aimPIDfeed(-3.06))));



  }

  // /**
  //  * Use this to pass the autonomous command to the main {@link Robot} class.
  //  *
  //  * @return the command to run in autonomous
  //  */
  // public Command getAutonomousCommand() {
    
  //   return new PathPlannerAuto("ChoreoTest2");

  // }

  public Superstructure getSuperstructure(){
    return superstructure;
  }

  public Drive getDrive(){
    return drive;
  }

  public Flywheel getFlywheel(){
    return flywheel;
  }

  // public ClimberSubsystem getClimberSubsystem(){
  //   return climberSubsystem;
  // }

  public Intake getIntake(){
    return intake;
  }


}
