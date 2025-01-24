package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.moandjiezana.toml.Toml;

import edu.wpi.first.math.geometry.Rotation2d;

public class CanEncoder extends TurnEncoder {
    private final CANcoder cancoder;
    private final Rotation2d zeroRotation;

    public CanEncoder(Toml toml) {
        cancoder = new CANcoder(toml.getLong("canid").intValue(), "rio");
        zeroRotation = new Rotation2d(toml.getDouble("zero_rotation"));
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble()).minus(zeroRotation);
    }
    
}
