// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorStructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.swerve.Drivetrain;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    @SuppressWarnings("unused")
    private Command autonomousCommand;
    private final Superstructure superstructure;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    public Robot() {
        superstructure = new Superstructure();
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
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
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void robotInit() {
        Drivetrain driveTrain = new Drivetrain();
        ElevatorStructure elevatorStructure = new ElevatorStructure();
        autonomousCommand = new SequentialCommandGroup(
                (driveTrain.new DefaultDrive(
                        () -> 1.0,
                        () -> 0.3 * 0.0,
                        () -> 0.1 * 0.0).alongWith(elevatorStructure.getL3PrepareCommand()))
                        .withDeadline(new WaitCommand(4)),
                elevatorStructure.getL3ScoreCommand()
            );
    }

    @Override
    public void autonomousInit() {
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }
}
