package frc.robot.subsystems.drive;

import com.moandjiezana.toml.Toml;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SonicSwerveModule extends SwerveModule {
    private final Translation2d position;
    private final DriveMotor driveMotor;
    private final TurnMotor turnMotor;
    private final DriveEncoder[] driveEncoders;
    private final TurnEncoder[] turnEncoders;
    private SwerveModuleState targetState = null;
    
    public SonicSwerveModule(Toml toml, Toml defaultToml) {
        setName(toml.getString("name", defaultToml.getString("name")));
        position = new Translation2d(
            toml.getDouble("x", 0.0),
            toml.getDouble("y", 0.0)
        );

        driveMotor = DriveMotor.create(toml.getTable("drive_motor"), defaultToml.getTable("drive_motor"));
        turnMotor = TurnMotor.create(toml.getTable("turn_motor"), defaultToml.getTable("turn_motor"));

        var driveEncoderTomls = toml.getTables("drive_encoders");
        var defaultDriveEncoders = defaultToml.getTables("drive_encoders");
        
        var turnEncoderTomls = toml.getTables("turn_encoders");
        var defaultTurnEncoders = defaultToml.getTables("turn_encoders");

        driveEncoders = new DriveEncoder[driveEncoderTomls.size()];

        for (int i = 0; i < driveEncoders.length; i++) {
            var type = driveEncoderTomls.get(i).getString("type", defaultDriveEncoders.get(i).getString("type"));
            if (type.equals("sparkmax")) {
                driveEncoders[i] = new SparkMaxDriveMotor.Encoder((SparkMaxDriveMotor)driveMotor);
            } else {
                throw new IllegalArgumentException(String.format("Unsupported Drive Encoder type: %s", type));
            }
        }

        turnEncoders = new TurnEncoder[turnEncoderTomls.size()];

        for (int i = 0; i < turnEncoders.length; i++) {
            var type = turnEncoderTomls.get(i).getString("type", defaultTurnEncoders.get(i).getString("type"));
            if (type.equals("sparkmax")) {
                turnEncoders[i] = new SparkMaxTurnMotor.Encoder((SparkMaxTurnMotor)turnMotor);
            } else if (type.equals("cancoder")) {
                turnEncoders[i] =  new CanEncoder(new Toml(defaultTurnEncoders.get(i)).read(turnEncoderTomls.get(i)));
            } else {
                throw new IllegalArgumentException(String.format("Unsupported Drive Encoder type: %s", type));
            }
        }

        addChild("Drive Motor", driveMotor);
        addChild("Turn Motor", turnMotor);

        for (int i = 0; i < driveEncoders.length; i++) {
            addChild(String.format("Drive Encoder %d", i), driveEncoders[i]);
        }
        
        for (int i = 0; i < turnEncoders.length; i++) {
            addChild(String.format("Turn Encoder %d", i), turnEncoders[i]);
        }
    }

    @Override
    public void periodic() {
        if (targetState != null) {
            driveMotor.setSpeed(targetState.speedMetersPerSecond);
            turnMotor.setSpeed(targetState.angle.minus(getAngle()).getRadians() * 5.0);
        }
    }

    @Override
    public Rotation2d getAngle() {
        return turnEncoders[0].getAngle();
        // return new Rotation2d(Arrays.stream(turnEncoders).mapToDouble(e -> e.getAngle().getRadians()).average().orElseThrow());
    }

    @Override
    public double getVelocity() {
        return targetState != null ? targetState.speedMetersPerSecond : 0.0;
    }

    @Override
    public void setTargetClosed(Translation2d translation) {
        targetState = new SwerveModuleState(translation.getNorm(), translation.getAngle());
    }

    @Override
    public void setTargetClosed(SwerveModuleState state) {
        targetState = state;
    }

    @Override
    public void setDriveOpen(double metersPerSec) {
        driveMotor.setSpeed(metersPerSec);
    }

    @Override
    public void setTurnOpen(double radPerSec) {
        turnMotor.setSpeed(radPerSec);
    }

    @Override
    public Translation2d getPosition() {
        return position;
    }
}
