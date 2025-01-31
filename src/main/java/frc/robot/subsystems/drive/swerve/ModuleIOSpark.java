package frc.robot.subsystems.drive.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ModuleIOSpark implements ModuleIO {
    private final SparkBase driveSpark;
    private final SparkBase turnSpark;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    @SuppressWarnings("unused")
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    private final CANcoder cancoder;

    private final Rotation2d zeroRotation;
    private Rotation2d angleOffset = null;

    public static class Config implements Cloneable {
        public static class Drive implements Cloneable {
            public int canid;
            public MotorType motorType;

            public IdleMode idleMode = IdleMode.kBrake;
            public boolean inverted = false;
            public int stallLimit;
            public int freeLimit;

            public double gearRatio;

            public Object clone() throws CloneNotSupportedException {
                return super.clone();
            }
        }
        
        public static class Turn implements Cloneable {
            public int canid;
            public MotorType motorType;

            public IdleMode idleMode = IdleMode.kBrake;
            public boolean inverted = false;
            public int stallLimit;
            public int freeLimit;
            
            public double zeroRotationRad;
            public double gearRatio;
            
            public Object clone() throws CloneNotSupportedException {
                return super.clone();
            }
        }

        public static class Cancoder implements Cloneable {
            public int canid;
            public double zeroRotationRad;
            
            public Object clone() throws CloneNotSupportedException {
                return super.clone();
            }
        }
        
        public Drive drive = new Drive();
        public Turn turn = new Turn();
        public Cancoder cancoder = new Cancoder();

        @Override
        public Object clone() throws CloneNotSupportedException {
            Config out = new Config();

            out.drive = (Drive)drive.clone();
            out.turn = (Turn)turn.clone();
            out.cancoder = (Cancoder)cancoder.clone();

            return out;
        }
    }

    public ModuleIOSpark(Config config) {
        driveSpark = new SparkMax(config.drive.canid, MotorType.kBrushless);
        turnSpark = new SparkMax(config.turn.canid, MotorType.kBrushless);

        driveEncoder = driveSpark.getEncoder();
        turnEncoder = turnSpark.getEncoder();

        driveController = driveSpark.getClosedLoopController();
        turnController = turnSpark.getClosedLoopController();

        var driveConfig = new SparkMaxConfig();

        driveConfig
            .inverted(config.drive.inverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(config.drive.stallLimit, config.drive.freeLimit)
            .voltageCompensation(12.0);
        
        driveConfig.encoder
            .positionConversionFactor(1 / config.drive.gearRatio * 2 * Math.PI)         // Rotations -> Radians
            .velocityConversionFactor(1 / config.drive.gearRatio * 2 * Math.PI / 60.0); // Rotations -> Radians
        
        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(2.0, 0.0, 0.0, 0.0);
        
        driveConfig.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(5)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        driveSpark.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        var turnConfig = new SparkMaxConfig();

        turnConfig
            .inverted(config.turn.inverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(config.turn.stallLimit, config.turn.freeLimit)
            .voltageCompensation(12.0);
        
        turnConfig.encoder
            .positionConversionFactor(1 / config.turn.gearRatio * 2 * Math.PI)         // Rotations -> Radians
            .velocityConversionFactor(1 / config.turn.gearRatio * 2 * Math.PI / 60.0); // Rotations per Minute -> Radians per Second
        
        turnConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(2.0, 0.0, 0.0, 0.0);
        
        turnConfig.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(5)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
            
        turnSpark.configure(turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        if (config.cancoder.canid == 0) {
            cancoder = null;
            zeroRotation = new Rotation2d(config.turn.zeroRotationRad);
        } else {
            cancoder = new CANcoder(config.cancoder.canid, "rio");
            zeroRotation = new Rotation2d(config.cancoder.zeroRotationRad);
        }

        driveSpark.setVoltage(0.0);
        turnSpark.setVoltage(0.0);
    }

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.driveConnected = true;
        inputs.drivePositionRad = driveEncoder.getPosition();
        inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
        inputs.driveAppliedVolts = driveSpark.getAppliedOutput() * driveSpark.getBusVoltage();
        inputs.driveCurrentAmps = driveSpark.getOutputCurrent();
        
        inputs.turnConnected = true;
        inputs.turnPositionRad = turnEncoder.getPosition();
        inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
        inputs.turnAppliedVolts = turnSpark.getAppliedOutput() * turnSpark.getBusVoltage();
        inputs.turnCurrentAmps = turnSpark.getOutputCurrent();

        if (cancoder != null) {
            if (cancoder.isConnected() && angleOffset == null) {
                var absAngle = Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble()).minus(zeroRotation);
                angleOffset = Rotation2d.fromRadians(inputs.turnPositionRad).minus(absAngle);
                inputs.turnZeroRotation = new Rotation2d(angleOffset.getMeasure());
            }

            // inputs.turnPosition = Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble()).minus(zeroRotation);
            inputs.turnPosition = Rotation2d.fromRadians(inputs.turnPositionRad).minus(angleOffset != null ? angleOffset : new Rotation2d(0.0));
        } else {
            inputs.turnPosition = Rotation2d.fromRadians(inputs.turnPositionRad).minus(zeroRotation);
            inputs.turnZeroRotation = new Rotation2d(zeroRotation.getMeasure());
        }

        inputs.turnVelocity = Rotation2d.fromRadians(inputs.turnVelocityRadPerSec);
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveSpark.setVoltage(MathUtil.clamp(voltage, -6.0, 6.0));
    }

    @Override
    public void setTurnVoltage(double voltage) {
        turnSpark.setVoltage(MathUtil.clamp(voltage, -6.0, 6.0));
    }

    @Override
    public void setDriveVelocity(double radPerSec) {
        setDriveVoltage(0.25 * radPerSec);
        // driveController.setReference(radPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
    
    @Override
    public void setTurnVelocity(double radPerSec) {
        setTurnVoltage(4.0 * radPerSec);
        // double ffVolts = 0.1 * radPerSec;
        // turnController.setReference(radPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setTurnPosition(double rad) {
        turnController.setReference(rad, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
}
