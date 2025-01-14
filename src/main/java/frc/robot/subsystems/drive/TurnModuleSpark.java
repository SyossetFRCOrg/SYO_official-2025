package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkRelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import jakarta.ws.rs.NotSupportedException;

public class TurnModuleSpark implements TurnModule {
    public static class Constants {
        public final double turnMotorReduction;
        public final double turnEncoderPositionFactor;
        public final Rotation2d zeroRotation;

        public Constants(
                double turnMotorReduction,
                double turnEncoderPositionFactor,
                Rotation2d zeroRotation) {
            this.turnMotorReduction = turnMotorReduction;
            this.turnEncoderPositionFactor = turnEncoderPositionFactor;
            this.zeroRotation = zeroRotation;
        }

        public static class Builder {
            private double turnMotorReduction;
            private Rotation2d zeroRotation;

            public Builder setTurnMotorReduction(double turnMotorReduction) {
                this.turnMotorReduction = turnMotorReduction;
                return this;
            }

            public Builder setZeroRotation(Rotation2d zeroRotation) {
                this.zeroRotation = zeroRotation;
                return this;
            }

            public Constants build() {
                return new Constants(
                    turnMotorReduction,
                    turnMotorReduction / (2 * Math.PI),
                    zeroRotation);
            }
        }
    }

    private final SparkBase spark;
    private final RelativeEncoder encoder;
    private final CANcoder cancoder;

    private final Constants constants;

    Rotation2d targetAngle = new Rotation2d(0);
    Rotation2d currentAngle = new Rotation2d(0);
    Rotation2d absoluteAngle = new Rotation2d(0);
    Rotation2d angleOffset = null;

    public TurnModuleSpark(SparkBase spark, CANcoder cancoder, Constants constants) {
        this.spark = spark;
        this.encoder = spark.getEncoder();

        this.cancoder = cancoder;

        this.constants = constants;

        spark.setVoltage(0);
    }

    @Override
    public void periodic() {
        currentAngle = new Rotation2d(encoder.getPosition() / constants.turnEncoderPositionFactor).minus(constants.zeroRotation);
        absoluteAngle = Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble()).minus(constants.zeroRotation);

        if (angleOffset == null && absoluteAngle.getRadians() != 0) {
            angleOffset = absoluteAngle.minus(currentAngle);
        }

        double diff = targetAngle.minus(getAngle()).getRadians();

        // TODO: replace w/ full PID controller
        if (Math.abs(diff) > 1) {
            setSpeed(Math.signum(diff) * 5.0);
        } else {
            setSpeed(diff * 5.0);
        }
    }

    @Override
    public Rotation2d getAngle() {
        return angleOffset == null ? new Rotation2d() : currentAngle.plus(angleOffset);
    }

    @Override
    public void setSpeed(double radPerSec) {
        spark.set(MathUtil.clamp(radPerSec, -5.0, 5.0) / 5.0 * 0.6);
    }

    @Override
    public void setTarget(Rotation2d targetAngle) {
        this.targetAngle = targetAngle;
    }
    
}
