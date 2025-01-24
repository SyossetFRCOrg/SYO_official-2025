package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class GyroSensor extends SubsystemBase {
    public abstract Rotation3d getAngle();
}
