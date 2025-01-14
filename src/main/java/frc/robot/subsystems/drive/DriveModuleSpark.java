package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;

public class DriveModuleSpark implements DriveModule {
    public static class Constants {
        public final double maxSpeedMetersPerSec;
        public final double wheelRadiusMeters;
        
        public final double kv;

        public Constants(
                double maxSpeedMetersPerSec,
                double wheelRadiusMeters,
                double kv) {
            this.maxSpeedMetersPerSec = maxSpeedMetersPerSec;
            this.wheelRadiusMeters = wheelRadiusMeters;

            this.kv = kv;
        }

        public static class Builder {
            private double maxSpeedMetersPerSec = 0;
            private double wheelRadiusMeters = 0;
            private double volts = 12;

            public Constants build() {
                return new Constants(
                        (5600.0 / 60.0) / ((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)) * wheelRadiusMeters * 2 * Math.PI, // Where does this number come from ???
                        wheelRadiusMeters,
                        volts / maxSpeedMetersPerSec * wheelRadiusMeters);
            }

            public Builder setMaxSpeedMetersPerSec(double maxSpeedMetersPerSec) {
                this.maxSpeedMetersPerSec = maxSpeedMetersPerSec;
                return this;
            }

            public Builder setWheelRadiusMeters(double wheelRadiusMeters) {
                this.wheelRadiusMeters = wheelRadiusMeters;
                return this;
            }

            public Builder setVolts(double volts) {
                this.volts = volts;
                return this;
            }
        }
    };

    private final SparkBase spark;
    private final Constants constants;

    public DriveModuleSpark(SparkBase spark, Constants constants) {
        this.spark = spark;
        this.constants = constants;

        spark.setVoltage(0);
    }

    @Override
    public void setSpeed(double metersPerSec) {
        spark.set(MathUtil.clamp(metersPerSec, -3.0, 3.0)/3.0 * 0.2);
    }
}
