package frc.robot.subsystems.drive;

import com.moandjiezana.toml.Toml;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class TurnMotor extends SubsystemBase {
    public static TurnMotor create(Toml toml, Toml defaultToml) {
        var type = toml.getString("type", defaultToml.getString("type"));
        if (type.equals("sparkmax")) {
            return new SparkMaxTurnMotor(toml, defaultToml);
        } else {
            throw new IllegalArgumentException(String.format("Unsupported Turn motor type: %s", type));
        }
    }

    public abstract void setSpeed(double radPerSec);
}
