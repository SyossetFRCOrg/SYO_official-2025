package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

@Deprecated
public class SparkMaxSwerveModule implements SwerveModule {
    public static final double wheelRadiusMeters = 12;
    public static final double driveKv = 0.1;
    public static final double turnEncoderPositionFactor = 150.0 / 7.0 / (2 * Math.PI);

    private final Rotation2d zeroRotation;

    private final SparkBase driveSpark;
    private final SparkBase turnSpark;
    
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final CANcoder cancoder;

    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    private Rotation2d turnRelativeOffset = null;
    private Rotation2d turnAbsolutePosition = new Rotation2d();
    private Rotation2d turnPosition = new Rotation2d();

    public SparkMaxSwerveModule(int driveCanId, int turnCanId, int encoderCanId) {
        zeroRotation = new Rotation2d(0.0, 0.0);

        driveSpark = new SparkFlex(driveCanId, MotorType.kBrushless);
        turnSpark = new SparkFlex(turnCanId, MotorType.kBrushless);

        driveEncoder = driveSpark.getEncoder();
        turnEncoder = turnSpark.getEncoder();

        cancoder = new CANcoder(encoderCanId, "rio");

        driveController = driveSpark.getClosedLoopController();
        turnController = turnSpark.getClosedLoopController();

        var driveConfig = new SparkMaxConfig();
        driveConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
            .voltageCompensation(12.0);
        driveConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        driveSpark.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        var turnConfig = new SparkMaxConfig();
        turnConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20)
            .voltageCompensation(12.0);
        turnConfig
            .signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / 50.0))
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        cancoder.getConfigurator().apply(new CANcoderConfiguration());
    }

    @Override
    public void periodic() {
        turnPosition = new Rotation2d(turnEncoder.getPosition() / turnEncoderPositionFactor).minus(zeroRotation);
        turnAbsolutePosition = Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble())
            .minus(zeroRotation);
        if (turnRelativeOffset == null && turnAbsolutePosition.getRadians() != 0.0) {
            turnRelativeOffset = turnAbsolutePosition.minus(turnPosition);
        }       
    }

    public void setDriveOpen(double output) {
        driveSpark.set(output);
    }

    public void setTurnOpen(double output) {
        turnSpark.set(output);
    }

    @Override
    public void setTargetClosed(Translation2d translation) {

    }

    @Override
    public void setTargetClosed(SwerveModuleState state) {
        setDriveVelocity(state.speedMetersPerSecond / wheelRadiusMeters);
        double target = state.angle.getRadians();
        double current = getAngle().getRadians();
        double diff = target - current;
        if (diff > Math.PI) {
            diff -= 2 * Math.PI;
        } else if (diff < -Math.PI) {
            diff += 2 * Math.PI;
        }
        
        if (Math.abs(diff) > 0.2) {
            setTurnVelocity(4.0 * Math.signum(diff));
        }
    }
    
    public void setDriveVelocity(double velocityMetersPerSec) {
        double vel = MathUtil.clamp(velocityMetersPerSec, -3.0, 3.0);
        driveSpark.set(vel / 3.0 * 0.2);
    }

    public void setTurnVelocity(double velocityRadPerSec) {
        double vel = MathUtil.clamp(velocityRadPerSec, -4.0, 4.0);
        driveSpark.set(vel / 4.0 * 0.2);
    }
    
    public void setTurnPosition(Rotation2d rotation) {
        double rad = rotation.getRadians();
        rad -= Math.floor(rad / (2 * Math.PI)) * 2 * Math.PI;
        turnController.setReference(rad, ControlType.kPosition);
    }

    @Override
    public Rotation2d getAngle() {
        return (turnRelativeOffset == null) ? new Rotation2d() : turnPosition.plus(turnRelativeOffset);
    }
}
