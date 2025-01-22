package frc.robot.subsystems.drive;

import com.moandjiezana.toml.Toml;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.TomlUtil;

public class SparkMaxTurnMotor extends TurnMotor {
    public static class Encoder extends TurnEncoder {
        private final RelativeEncoder encoder;
        
        public Encoder(SparkMaxTurnMotor motor) {
            encoder = motor.spark.getEncoder();
        }

        @Override
        public Rotation2d getAngle() {
            return new Rotation2d(encoder.getPosition());
        }
    }

    private final SparkBase spark;

    public SparkMaxTurnMotor(Toml toml) {
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
    }

    @Override
    public void setSpeed(double radPerSec) {
        spark.setVoltage(MathUtil.clamp(radPerSec, -5.0, 5.0) / 5.0 * 0.7);
    }
}
