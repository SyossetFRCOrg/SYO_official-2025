package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class TurnEncoder extends SubsystemBase {
    public abstract Rotation2d getAngle();
}
