package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autos.AutoChooser;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private AutoChooser autoChooser;

  private final RobotContainer robotContainer;

  public Robot() {
    robotContainer = new RobotContainer();
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();

    autoChooser =
        AutoChooser.create(
            robotContainer, robotContainer.getDrive(), robotContainer.getSuperstructure());
    Shuffleboard.getTab("Autonomous")
        .add("Auto Program", autoChooser)
        .withSize(6, 3)
        .withPosition(12, 0)
        .withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

    autoChooser.reset("SmartDashboard/Autonomous/2025Programs");
    // robotContainer.getSuperstructure().setWantedSuperState(SuperState.STOPPED);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    autoChooser.update();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // autonomousCommand = robotContainer.getAutonomousCommand();

    // // schedule the autonomous command (example)
    // if (autonomousCommand != null) {
    //   autonomousCommand.schedule();
    // }

    autoChooser.getSelectedCommand().ifPresent(CommandScheduler.getInstance()::schedule);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
