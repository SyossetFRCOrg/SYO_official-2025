package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Climber.*;

@LoggedObject
public class Climber extends SubsystemBase {
    @Log
    private final CANSparkMax motor;
    @Log
    private boolean initialized;

    public Climber() {
        motor = SparkConfigurator.createSparkMax(MOTOR_ID, MotorType.kBrushless, MOTOR_INVERTED,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0));
        motor.getEncoder().setPosition(0);
    }

    public boolean getInitialized() {
        return initialized;
    }

    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -4, 4);
        // Utils.applySoftStops(voltage, getPosition(), MIN_POSITION_METERS,
        // MAX_POSITION_METERS);
        motor.setVoltage(voltage);
    }

    public Command winchUpCommand() {
        return Commands.startEnd(
                () -> setVoltage(12),
                () -> setVoltage(0))
                .withName("climber.winchUp");
    }

    public Command winchDownCommand() {
        return Commands.startEnd(
                () -> setVoltage(12),
                () -> setVoltage(0))
                .withName("climber.winchDown");
    }

    public void resetPosition() {
        motor.getEncoder().setPosition(0);
        initialized = true;
    }

    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("arm.resetPosition");
    }
}
