package frc.robot;

import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDState;

@LoggedObject
public class HoundBrian {
    @Log
    private final DigitalInput drivetrainButton = new DigitalInput(0);
    @Log
    private final DigitalInput elevatorButton = new DigitalInput(1);
    @Log
    private final DigitalInput armButton = new DigitalInput(2);
    @Log
    private final DigitalInput climberButton = new DigitalInput(3);
    @Log
    private final DigitalInput unusedButton2 = new DigitalInput(4);
    @Log
    private final DigitalInput switch1 = new DigitalInput(5);
    @Log
    private final DigitalInput switch2 = new DigitalInput(6);

    public HoundBrian(Drivetrain drivetrain, Elevator elevator, Arm arm, Climber climber, LEDs leds) {

        new Trigger(drivetrainButton::get).negate()
                .and(DriverStation::isDisabled)
                .onTrue(drivetrain.resetGyroCommand().ignoringDisable(true));
        new Trigger(elevatorButton::get).negate()
                .and(DriverStation::isDisabled)
                .onTrue(elevator.resetPositionCommand().ignoringDisable(true));
        new Trigger(armButton::get).negate()
                .and(DriverStation::isDisabled)
                .onTrue(arm.resetPositionCommand().ignoringDisable(true));
        new Trigger(climberButton::get).negate()
                .and(DriverStation::isDisabled)
                .onTrue(climber.resetPositionCommand().ignoringDisable(true));

        new Trigger(switch1::get)
                .whileTrue(leds.requestStateCommand(LEDState.DEMO_RED).ignoringDisable(true));
        new Trigger(switch2::get)
                .whileTrue(leds.requestStateCommand(LEDState.DEMO_GOLD).ignoringDisable(true));
    }
}
