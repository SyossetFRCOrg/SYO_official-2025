package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class TurnModuleSpark extends TurnModule {
    public static class Constants {
        public final double turnMotorReduction;
        public final double turnEncoderPositionFactor;
        public final Rotation2d zeroRotation;

        public final double kp;
        public final double ki;
        public final double kd;

        public Constants(
                double turnMotorReduction,
                double turnEncoderPositionFactor,
                Rotation2d zeroRotation,
                double kp,
                double ki,
                double kd) {
            this.turnMotorReduction = turnMotorReduction;
            this.turnEncoderPositionFactor = turnEncoderPositionFactor;
            this.zeroRotation = zeroRotation;
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }

        public static class Builder {
            private double turnMotorReduction;
            private Rotation2d zeroRotation;
            private double kp;
            private double ki;
            private double kd;

            public Builder setTurnMotorReduction(double turnMotorReduction) {
                this.turnMotorReduction = turnMotorReduction;
                return this;
            }

            public Builder setZeroRotation(Rotation2d zeroRotation) {
                this.zeroRotation = zeroRotation;
                return this;
            }

            public Builder setPID(double kp, double ki, double kd) {
                this.kp = kp;
                this.ki = ki;
                this.kd = kd;
                return this;
            }

            public Constants build() {
                return new Constants(
                    turnMotorReduction,
                    turnMotorReduction / (2 * Math.PI),
                    zeroRotation,
                    kp, ki, kd);
            }
        }
    }

    private final SparkBase spark;
    private final RelativeEncoder encoder;
    private final CANcoder cancoder;

    private final Constants constants;
    private final PIDController pid;

    Rotation2d targetAngle = new Rotation2d(0);
    Rotation2d currentAngle = new Rotation2d(0);
    Rotation2d absoluteAngle = new Rotation2d(0);
    Rotation2d angleOffset = null;

    public TurnModuleSpark(SparkBase spark, CANcoder cancoder, Constants constants) {
        this.spark = spark;
        this.encoder = spark.getEncoder();

        this.cancoder = cancoder;

        this.constants = constants;
        this.pid = new PIDController(constants.kp, constants.ki, constants.kd);
        this.pid.enableContinuousInput(-Math.PI, Math.PI);

        spark.setVoltage(0);
    }

    @Override
    public void periodic() {
        currentAngle = Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble()).minus(constants.zeroRotation);

        setVoltage(pid.calculate(getAngle().getRadians(), targetAngle.getRadians()));
    }

    @Override
    public Rotation2d getAngle() {
        return currentAngle;
    }

    @Override
    public void setSpeed(double radPerSec) {
        spark.set(MathUtil.clamp(radPerSec, -5.0, 5.0) / 5.0 * 0.6);
    }

    @Override
    public void setVoltage(double volts) {
        spark.setVoltage(MathUtil.clamp(volts, -6.0, 6.0));
    }

    @Override
    public void setTarget(Rotation2d targetAngle) {
        this.targetAngle = targetAngle;
    }
    
}
