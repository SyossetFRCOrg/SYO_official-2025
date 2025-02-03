package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class MotorIOSpark implements MotorIO, Sendable {
    public static class Config {
        public int canid;
        public MotorType motorType;

        public IdleMode idleMode = IdleMode.kBrake;
        public boolean inverted = false;
        public int stallLimit;
        public int freeLimit;

        public double gearRatio;
        public int frequency = 200;

        public double minOutput = -1.0;
        public double maxOutput = 1.0;

        public double kP;
        public double kI;
        public double kD;

        public double kS;
        public double kV;
        public double kA;
        public double kG;

        public double maxVelocity;
        public double maxAcceleration;

        public FeedForwardType feedForward = FeedForwardType.NONE;
    }

    private final SparkBase spark;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;

    private double kS;
    private double kV;
    private double kA;
    private double kG;

    private FeedForwardType feedForward;

    public MotorIOSpark(Config config) {
        spark = new SparkMax(config.canid, config.motorType);
        encoder = spark.getEncoder();
        controller = spark.getClosedLoopController();

        var sparkConfig = new SparkMaxConfig();
        
        sparkConfig
            .inverted(config.inverted)
            .idleMode(config.idleMode)
            .smartCurrentLimit(config.stallLimit, config.freeLimit)
            .voltageCompensation(12.0);
        
        sparkConfig.encoder
            .positionConversionFactor(1 / config.gearRatio * 2 * Math.PI)
            .positionConversionFactor(1 / config.gearRatio * 2 * Math.PI / 60.0);
        
        sparkConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(config.minOutput, config.maxOutput, ClosedLoopSlot.kSlot0)
            .outputRange(config.minOutput, config.maxOutput, ClosedLoopSlot.kSlot1)
            .pidf(config.kP, config.kI, config.kD, 0.0, ClosedLoopSlot.kSlot0)
            .pidf(0.0, 0.0, 0.0, 0.0, ClosedLoopSlot.kSlot1);
        
        sparkConfig.closedLoop.maxMotion
            .maxAcceleration(config.maxAcceleration, ClosedLoopSlot.kSlot0)
            .maxAcceleration(config.maxAcceleration, ClosedLoopSlot.kSlot1)
            .maxVelocity(config.maxVelocity, ClosedLoopSlot.kSlot0)
            .maxVelocity(config.maxVelocity, ClosedLoopSlot.kSlot1);
        
        sparkConfig.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(1000 / config.frequency)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(1000 / config.frequency)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        
        spark.configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        spark.setVoltage(0.0);

        kS = config.kS;
        kV = config.kV;
        kA = config.kA;
        kG = config.kG;

        feedForward = config.feedForward;
    }

    @Override
    public void updateInputs(Inputs inputs) {
        if (spark.getLastError() != REVLibError.kOk) {
            inputs.connected = false;
            return;
        }

        inputs.connected = true;
        inputs.positionRad = encoder.getPosition();
        inputs.velocityRadPerSec = encoder.getVelocity();
        inputs.appliedVolts = spark.getAppliedOutput() * spark.getBusVoltage();
        inputs.currentAmps = spark.getOutputCurrent();
    }

    @Override
    public void setVoltage(double voltage) {
        spark.setVoltage(voltage);
    }

    @Override
    public void setVelocity(double velocity) {
        double ffVolts = kS * Math.signum(velocity) + kV * velocity;
        ffVolts += switch (feedForward) {
            case NONE, DRIVE -> 0.0;
            case ELEVATOR -> kG;
            case ARM -> kG * Math.cos(encoder.getPosition());
        };
        controller.setReference(velocity, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1, ffVolts, ArbFFUnits.kVoltage);
    }

    @Override
    public void setSetpoint(double position) {
        controller.setReference(position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setPosition(double rad) {
        encoder.setPosition(rad);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("kS", () -> kS, x -> kS = x);
        builder.addDoubleProperty("kV", () -> kV, x -> kV = x);
        builder.addDoubleProperty("kA", () -> kA, x -> kA = x);
        builder.addDoubleProperty("kG", () -> kG, x -> kG = x);
    }
}
