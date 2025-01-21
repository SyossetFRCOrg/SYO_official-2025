package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class TurnModule extends SubsystemBase {
    public static enum ControlMode {
        SETPOINT,
        MANUAL
    }

    public abstract Rotation2d getAngle();
    public abstract void setSpeed(double radPerSec);
    public abstract void setVoltage(double volts);

    public abstract void setTarget(Rotation2d rad);
}
