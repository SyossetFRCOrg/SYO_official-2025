package frc.robot.subsystems.drive;

import com.moandjiezana.toml.Toml;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class TurnMotor extends SubsystemBase {
    public static TurnMotor create(Toml toml) {
        var type = toml.getString("type");
        if (type.equals("sparkmax")) {
            return new SparkMaxTurnMotor(toml);
        } else {
            throw new IllegalArgumentException(String.format("Unsupported Turn motor type: %s", type));
        }
    }

    public abstract void setSpeed(double radPerSec);
}
