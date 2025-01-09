package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Intake.*;

@LoggedObject
public class Intake extends SubsystemBase implements BaseIntake {
    @Log
    private final CANSparkFlex motor;

    public Intake() {
        motor = SparkConfigurator.createSparkFlex(MOTOR_ID, MotorType.kBrushless,
                MOTOR_INVERTED,
                (s) -> s.setIdleMode(IdleMode.kCoast),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT));
    }

    public void setRollerVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public Command runRollersCommand() {
        return Commands.startEnd(
                () -> setRollerVoltage(3),
                () -> setRollerVoltage(0))
                .withName("intake.runRollers");
    }

    @Override
    public Command reverseRollersCommand() {
        return Commands.startEnd(
                () -> setRollerVoltage(-12),
                () -> setRollerVoltage(0))
                .withName("intake.reverseRollers");
    }
}
