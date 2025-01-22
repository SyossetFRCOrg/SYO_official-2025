package frc.robot.subsystems.drive;

import com.moandjiezana.toml.Toml;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DriveMotor extends SubsystemBase {
    public static DriveMotor create(Toml toml) {
        var type = toml.getString("type");
        if (type.equals("sparkmax")) {
            return new SparkMaxDriveMotor(toml);
        } else {
            throw new IllegalArgumentException(String.format("Unsupported Drive motor type: %s", type));
        }
    }

    public abstract void setSpeed(double metersPerSec);
}
