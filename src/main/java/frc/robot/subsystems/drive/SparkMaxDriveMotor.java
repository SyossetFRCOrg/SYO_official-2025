package frc.robot.subsystems.drive;

import com.moandjiezana.toml.Toml;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.TomlUtil;

public class SparkMaxDriveMotor extends DriveMotor {
    public static class Encoder extends DriveEncoder {
        public Encoder(SparkMaxDriveMotor motor) {

        }
    }

    private final SparkBase spark;
    // private final double wheelRadius;

    public SparkMaxDriveMotor(Toml toml, Toml defaultToml) {
        var motorToml = new Toml(defaultToml).read(toml);

        MotorType motorType = TomlUtil.mapString(motorToml, "motor_type", "brushless", 
            new String[] {"brushless", "brushed"},
            new MotorType[] {MotorType.kBrushless, MotorType.kBrushed}
        );

        IdleMode idleMode = TomlUtil.mapString(motorToml, "idle_mode", "brake", 
            new String[] {"brake", "coast"},
            new IdleMode[] {IdleMode.kBrake, IdleMode.kCoast}
        );

        spark = new SparkMax(motorToml.getLong("canid").intValue(), motorType);

        var config = new SparkMaxConfig();

        config
            .inverted(motorToml.getBoolean("inverted", false))
            .idleMode(idleMode)
            .smartCurrentLimit(
                motorToml.getLong("stall_limit", 0L).intValue(),
                motorToml.getLong("free_limit", 0L).intValue()
            )
            .voltageCompensation(motorToml.getDouble("volts", 12.0));
        
        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // wheelRadius = toml.getDouble("wheel_radius", 0.04);
    }

    @Override
    public void setSpeed(double metersPerSec) {
        spark.set(MathUtil.clamp(metersPerSec, -4.0, 4.0) / 4.0 * 0.5);
    }
}
