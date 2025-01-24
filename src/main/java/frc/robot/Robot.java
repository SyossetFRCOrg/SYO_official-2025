// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.moandjiezana.toml.Toml;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.drive.DefaultDriveCommand;
import frc.robot.subsystems.drive.Drivetrain;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private final AlgaeIntakeSubsystem algaeIntake;
  private final Drivetrain drivetrain;
  private final CommandXboxController controller;
  private final ElevatorSubsystem elevator;
  private final ArmSubsystem arm;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.

    // drivetrain = new SonicSwerveDrivetrain("config/swerve.toml");
    var deployDir = Filesystem.getDeployDirectory();

    controller = new CommandXboxController(0);

    drivetrain = Drivetrain.create(new Toml().read(new File(deployDir, "config/swerve.toml")));
    drivetrain.setDefaultCommand(new DefaultDriveCommand(
      drivetrain, 
      () -> controller.getLeftY(), 
      () -> controller.getLeftX(), 
      () -> -controller.getRightX(), 
      5.0, 5.0
    ));

    algaeIntake = new AlgaeIntakeSubsystem();
    algaeIntake.setDefaultCommand(Commands.run(() -> algaeIntake.setRollerVoltage(0.0), algaeIntake));
    controller.leftBumper().whileTrue(Commands.run(() -> algaeIntake.setRollerVoltage(4.0), algaeIntake));
    controller.rightBumper().whileTrue(Commands.run(() -> algaeIntake.setRollerVoltage(-4.0), algaeIntake));

    elevator = new ElevatorSubsystem();
    elevator.setDefaultCommand(Commands.run(() -> elevator.setVoltage(0.0), elevator));
    controller.x().whileTrue(Commands.run(() -> elevator.setVoltage(2.0), algaeIntake));
    controller.y().whileTrue(Commands.run(() -> elevator.setVoltage(-1.0), algaeIntake));

    arm = new ArmSubsystem();
    arm.setDefaultCommand(Commands.run(() -> arm.setArmVoltage(0.0), algaeIntake));
    controller.a().whileTrue(Commands.run(() -> arm.setArmVoltage(0.5), algaeIntake));
    controller.b().whileTrue(Commands.run(() -> arm.setArmVoltage(-0.2), algaeIntake));
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }
}
