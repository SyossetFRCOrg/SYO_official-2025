package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import frc.robot.subsystems.LightsSubsystem.Color;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {
  // private Command autonomousCommand;
  // private AutoChooser autoChooser;

  // private final RobotContainer robotContainer;

  public static final int CANdleID = 35;

  private static final CANdle candle = new CANdle(CANdleID, "*");

  // Team colors
  public static Color orange = new Color(255, 25, 0);

  public Robot() {
    // robotContainer = new RobotContainer();
    // switch (Constants.currentMode) {
    //   case REAL:
    //     // Running on a real robot, log to a USB stick ("/U/logs")
    //     Logger.addDataReceiver(new WPILOGWriter());
    //     Logger.addDataReceiver(new NT4Publisher());
    //     break;

    //   case SIM:
    //     // Running a physics simulator, log to NT
    //     Logger.addDataReceiver(new NT4Publisher());
    //     break;

    //   case REPLAY:
    //     // Replaying a log, set up replay source
    //     setUseTiming(false); // Run as fast as possible
    //     String logPath = LogFileUtil.findReplayLog();
    //     Logger.setReplaySource(new WPILOGReader(logPath));
    //     Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    //     break;
    // }

    // Logger.start();

    // // autoChooser =
    // //     AutoChooser.create(
    // //         robotContainer, robotContainer.getDrive(), robotContainer.getSuperstructure());
    // Shuffleboard.getTab("Autonomous")
    //     .add("Auto Program", autoChooser)
    //     .withSize(6, 3)
    //     .withPosition(12, 0)
    //     .withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  @Override
  public void robotPeriodic() {
    // CommandScheduler.getInstance().run();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

    CANdleConfiguration candleConfiguration = new CANdleConfiguration();
    candleConfiguration.statusLedOffWhenActive = true;
    candleConfiguration.disableWhenLOS = false;
    candleConfiguration.stripType = LEDStripType.RGB;
    candleConfiguration.brightnessScalar = 1.0;
    candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(candleConfiguration, 100);
    candle.configBrightnessScalar(.75, 100);

    // autoChooser.reset("SmartDashboard/Autonomous/2025Programs");
    // robotContainer.getSuperstructure().setWantedSuperState(SuperState.STOPPED);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    candle.setLEDs(orange.red, orange.green, orange.blue);
    // autoChooser.update();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // autonomousCommand = robotContainer.getAutonomousCommand();

    // // schedule the autonomous command (example)
    // if (autonomousCommand != null) {
    //   autonomousCommand.schedule();
    // }

    // autoChooser.getSelectedCommand().ifPresent(CommandScheduler.getInstance()::schedule);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // if (autonomousCommand != null) {
    //   autonomousCommand.cancel();
    // }
  }

  @Override
  public void teleopPeriodic() {
    candle.setLEDs(orange.red, orange.green + 75, orange.blue);

    if (orange.red > 25) {
      orange = orange.dim(.99);
    } else {
      orange = orange.dim(5);
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    // CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
