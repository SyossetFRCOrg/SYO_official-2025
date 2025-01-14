package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public interface TurnModule {
    public static enum ControlMode {
        SETPOINT,
        MANUAL
    }

    public void periodic();

    public Rotation2d getAngle();
    public void setSpeed(double radPerSec);

    public void setTarget(Rotation2d rad);
}
