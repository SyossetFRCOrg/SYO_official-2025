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

    public SparkMaxDriveMotor(Toml toml) {
        MotorType motorType = TomlUtil.mapString(toml, "motor_type", "brushless", 
            new String[] {"brushless", "brushed"},
            new MotorType[] {MotorType.kBrushless, MotorType.kBrushed}
        );

        IdleMode idleMode = TomlUtil.mapString(toml, "idle_mode", "brake", 
            new String[] {"brake", "coast"},
            new IdleMode[] {IdleMode.kBrake, IdleMode.kCoast}
        );

        spark = new SparkMax(toml.getDouble("canid").intValue(), motorType);

        var config = new SparkMaxConfig();

        config
            .idleMode(idleMode)
            .smartCurrentLimit(
                toml.getDouble("stall_limit", 0.0).intValue(),
                toml.getDouble("free_limit", 0.0).intValue()
            )
            .voltageCompensation(toml.getDouble("volts", 12.0));
        
        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // wheelRadius = toml.getDouble("wheel_radius", 0.04);
    }

    @Override
    public void setSpeed(double metersPerSec) {
        spark.setVoltage(MathUtil.clamp(metersPerSec, -4.0, 4.0) / 4.0 * 0.7);
    }
}
