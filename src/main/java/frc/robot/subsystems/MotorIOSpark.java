package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorIOSpark implements MotorIO, Sendable {
    public static class Config {
        public int canid;
        public MotorType motorType;

        public IdleMode idleMode = IdleMode.kBrake;
        public boolean inverted = false;
        public int stallLimit;
        public int freeLimit;

        public double gearRatio;
        public int frequency = 500;

        public double minOutput = -1.0;
        public double maxOutput = 1.0;

        public double kP;
        public double kI;
        public double kD;

        public double vkP;
        public double vkI;
        public double vkD;

        public double maxVelocity;
        public double maxAcceleration;
        public double maxJerk;

        public FeedForward feedForward;

        public double debounceTime;
        public double tolerance;
    }

    private final SparkBase spark;
    private final RelativeEncoder encoder;
    private final ProfiledPIDController positionPid;
    private final ProfiledPIDController velocityPid;
    private final Notifier controller = new Notifier(this::runController);
    
    private ControlMode controlMode = ControlMode.NONE;

    private FeedForward feedForward;
    private double targetVelocity;
    private double targetVoltage;

    private final Debouncer debouncer;
    
    public MotorIOSpark(Config config) {
        spark = new SparkMax(config.canid, config.motorType);
        encoder = spark.getEncoder();
        // controller = spark.getClosedLoopController();

        var sparkConfig = new SparkMaxConfig();
        
        sparkConfig
            .inverted(config.inverted)
            .idleMode(config.idleMode)
            .smartCurrentLimit(config.stallLimit, config.freeLimit)
            .voltageCompensation(12.0);
        
        sparkConfig.encoder
            .positionConversionFactor(1 / config.gearRatio * 2 * Math.PI)
            .velocityConversionFactor(1 / config.gearRatio * 2 * Math.PI / 60.0);

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

        feedForward = config.feedForward;
        positionPid = new ProfiledPIDController(config.kP, config.kI, config.kD, new TrapezoidProfile.Constraints(config.maxVelocity, config.maxAcceleration), 1.0 / config.frequency);
        velocityPid = new ProfiledPIDController(config.vkP, config.vkI, config.vkD, new TrapezoidProfile.Constraints(config.maxAcceleration, config.maxJerk), 1.0 / config.frequency);

        controller.startPeriodic(1.0 / config.frequency);

        debouncer = new Debouncer(config.debounceTime);
        positionPid.setTolerance(config.tolerance);
    }

    public void putData(String name) {
        SmartDashboard.putData(name + "/Motor", this);
        SmartDashboard.putData(name + "/Position PID", positionPid);
        SmartDashboard.putData(name + "/Velocity PID", velocityPid);
        SmartDashboard.putData(name + "/Feed Forward", feedForward);
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
        targetVoltage = voltage;
        spark.setVoltage(voltage);
    }

    @Override
    public void setVelocity(double velocity) {
        velocityPid.reset(encoder.getVelocity());
        velocityPid.setGoal(velocity);
        targetVelocity = velocity;
        controlMode = ControlMode.VELOCITY;
    }

    @Override
    public void setSetpoint(double position) {
        positionPid.reset(encoder.getPosition());
        positionPid.setGoal(position);
        controlMode = ControlMode.POSITION;
    }

    public void runController() {
        switch (controlMode) {
            case NONE:
                break;
            case POSITION:
                setVoltage(positionPid.calculate(encoder.getPosition()));
                break;
            case VELOCITY:
                double ffVolts = feedForward != null ? feedForward.calculate(encoder.getPosition(), targetVelocity) : 0.0;
                setVoltage(velocityPid.calculate(encoder.getVelocity()) + ffVolts);
                break;
        }
    }

    @Override
    public void resetPosition(double rad) {
        encoder.setPosition(rad);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Voltage", () -> targetVoltage, volts -> setVoltage(volts));
        builder.addDoubleProperty("Velocity", () -> targetVelocity, vel -> setVelocity(vel));
    }

    @Override
    public boolean atSetpoint() {
        return debouncer.calculate(positionPid.atGoal());
    }
}
