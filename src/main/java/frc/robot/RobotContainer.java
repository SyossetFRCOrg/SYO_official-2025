// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.urcl.URCL;

import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.SendableLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class RobotContainer {
    @SendableLog
    private Mechanism2d mechanisms = new Mechanism2d(5, 3);
    private MechanismRoot2d root = mechanisms.getRoot("root", 2.5, 0.25);

    @SuppressWarnings("unused")
    private MechanismLigament2d fromRobot = root
            .append(new MechanismLigament2d("fromRobot", Units.inchesToMeters(5.5), 180, 0,
                    new Color8Bit(Color.kWhite)));
    @SuppressWarnings("unused")
    private MechanismLigament2d elevatorBase = root
            .append(new MechanismLigament2d("elevatorBase", Units.inchesToMeters(36), 90, 2,
                    new Color8Bit(Color.kWhite)));
    private MechanismLigament2d elevatorLigament = root
            .append(new MechanismLigament2d("elevatorStage", Units.inchesToMeters(10), 90,
                    4,
                    new Color8Bit(Color.kOrange)));
    private MechanismLigament2d armLigament = elevatorLigament
            .append(new MechanismLigament2d("armLigament", Units.inchesToMeters(10), 270,
                    5,
                    new Color8Bit(Color.kRed)));

    PositionTracker positionTracker = new PositionTracker();

    @Log
    Drivetrain drivetrain = new Drivetrain();
    @Log
    Elevator elevator = new Elevator(positionTracker, elevatorLigament);
    @Log
    Arm arm = new Arm(positionTracker, armLigament, elevator::getCarriageComponentPose);
    @Log
    Intake intake = new Intake();
    @Log
    Climber climber = new Climber();

    @Log
    CoralSim coralSim = new CoralSim(drivetrain::getPose, arm::getClawComponentPose);

    @Log
    LEDs leds = new LEDs();

    @Log
    HoundBrian houndbrian = new HoundBrian(drivetrain, elevator, arm, climber, leds);

    @Log
    private final Supplier<Boolean> initialized = GlobalStates.INITIALIZED::enabled;

    @SendableLog
    CommandScheduler scheduler = CommandScheduler.getInstance();

    @Log(groups = "gamePieces")
    public Pose3d getCoralPose() {
        Pose3d relativeCoralPose = arm.getClawComponentPose().plus(new Transform3d(0.143, 0, 0, new Rotation3d()));
        return new Pose3d(drivetrain.getPose())
                .plus(new Transform3d(relativeCoralPose.getTranslation(), relativeCoralPose.getRotation()))
                .plus(new Transform3d(0, 0, 0, new Rotation3d(0, Math.PI / 2.0, 0)));
    }

    public RobotContainer() {
        configureBindings();
        configureAuto();
        LoggingManager.getInstance().registerObject(this);
        SparkConfigurator.safeBurnFlash();
        URCL.start();

        new Trigger(() -> {
            return elevator.getInitialized()
                    && arm.getInitialized()
                    && climber.getInitialized();
        }).onTrue(GlobalStates.INITIALIZED.enableCommand());

        new Trigger(DriverStation::isEnabled)
                .onTrue(Commands.parallel(
                        elevator.resetControllersCommand(),
                        arm.resetControllersCommand()).withName("resetControllers"));
    }

    private void configureBindings() {
        Controls.configureControls(0, drivetrain, elevator, arm, intake, climber, coralSim);
        Controls.configureTestingControls(1, drivetrain, elevator, arm, intake, climber, leds);
    }

    public void configureAuto() {
        AutoManager.getInstance().addRoutine(Autos.testPath(drivetrain));
        AutoManager.getInstance().addRoutine(Autos.GDC(drivetrain, elevator, arm, coralSim));
    }
}
